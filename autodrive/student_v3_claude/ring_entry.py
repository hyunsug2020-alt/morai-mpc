"""Roundabout entry at the mandatory stop line (user rules 2026-10-10).

1. Stop at the line for 1 s (stop_line.StopLineController latches it).
2. Then enter right behind the rear of a ring car passing our entry, launching
   so that our nose meets the ring lane LEAD_GAP_S behind that car's rear.
3. Never cut in front of an approaching ring car: every car that would reach
   our conflict point after our nose must arrive there no sooner than
   NEXT_MARGIN_S after our rear has cleared it (conservative speed).
4. Once committed, follow only: decelerate behind what is on our path ahead,
   never a hold that waits for a gap.

follow-20261010T113545Z lap 1 (75 s at the entry, 4 twitching launches): every
aborted launch started 0.0-0.45 s wall *after* the passing car reached our
conflict point and the next ring car would have reached it 0.30 s before to
0.50 s after our rear; all five successful entries in three drives launched
0.7-1.0 s *before* the passing car got there (+1.8-2.8 s wall). Ring cars are
spawned every 5.0 s sim at (-111.3, 358.3) with 20 km/h; their measured speed
over the ring matched 20.7 km/h x RTF (sim-paced), 28-31 m apart.

Geometry (planner world frame; stations along our published path):
  contact: the ring lane centre comes within LANE_CONTACT_M of our path (our
           body would overlap a car on it); at the roundabout ~6.8 m past
           the line, where entry lane and ring lane converge;
  CP:      within CONFLICT_POINT_M (the merge/crossing point), ~9 m past it;
  exit:    a crossing lane leaves contact again (a merge never does).
Times are simulator seconds (ego physics); object velocities arrive already
converted (sim_time.to_sim_objects).
"""
from __future__ import annotations

import math

import numpy as np
from ego_body import EGO_FRONT_M, EGO_REAR_M

EGO_HALF_LENGTH_M = 2.3175
EGO_HALF_WIDTH_M = .946
# Lane centres this close: our body overlaps a ring car on that lane (its half
# width ~1.05 m + ours 0.946 + 0.4 m shell).
LANE_CONTACT_M = 2.4
# Lane centre this close to our path: the merge / crossing point (CP).
CONFLICT_POINT_M = 1.2
# A lane that leaves our corridor within this arc after CP crosses it; one that
# stays longer merges (at the roundabout the ring and our exit share ~20 m).
CROSSING_EXIT_M = 8.
# Our path searched for conflicts (stations relative to our centre).
CONFLICT_BEHIND_M = 1.
CONFLICT_AHEAD_M = 35.
# Lane paths are followed this far from a car's matched position.
LANE_LOOKUP_M = 70.
# Readiness and unknown-motion checks only count cars this close to our CP (ring
# arc) or this soon (s): the spawn point 52 m up the ring held us 1-3 frames per
# 6 s cycle in follow-20261010T113545Z.
RING_SCOPE_ARC_M = 35.
RING_SCOPE_ETA_S = 6.
UNKNOWN_RANGE_M = 30.          # without a map: straight distance to the ego
# Launch from standstill, simulator units: (acceleration m/s^2, dead time s).
# Measured at the release decision (first non-zero target, no target ramp):
# follow-20261010T130317Z first motion after 0.36-0.40 s sim (lap 1/2), then
# 2.5-2.7 m/s^2 sim (MPC log, target >= 5 km/h above the speed); 113545Z with the
# old ramp 2.85-3.1 m/s^2 after 0.22-0.26 s. The decision's own observation age adds
# ~0.1 s sim.
LAUNCH_SLOW = (2.5, .5)        # rear-clear time (next car margin): never faster than measured
LAUNCH_NOMINAL = (2.7, .4)     # nose-in time (follow the passing car's rear)
# Comfortable braking towards a curve ahead (speed_limit profiles).
CURVE_BRAKE_MPS2 = 2.
# The follower tracks a cap above the current speed at about this rate (1/s), up to the
# launch acceleration (MPC log of follow-20261010T130317Z: 1.8 m/s^2 for a 1 m/s gap).
LAUNCH_TRACK_GAIN = 2.
# A followed ring car's speed for our rear-clear time: its fitted speed / this. Ring cars
# slowed while merging ahead of us (follow-20261010T130317Z lap 1: 2.2-4 m/s along our path
# for 2 s at a fitted 5.5 m/s); a closer bound let the model promise 0.1-1.3 s too early.
# Against closed-loop replays of the logged ring traffic (test_ring_entry_replay: 10 entries,
# measured launch) the modelled rear-clear time was -0.17..+0.56 s off (one 0.17 s
# underestimate, that entry's next car still 2.2 s sim behind our rear).
LEADER_SPEED_RATIO = 1.3
LEAD_GAP_S = .5                # our nose meets the ring lane this long behind its rear
# ... and it is already passing in front of our entry (arc to our CP): a car still at the
# ring node 15 m up read 5.65 m/s wall (fit through its turn) for ~4.7 and merged in front
# of our nose (replay of follow-20261010T130317Z 22:12:09).
LEAD_ARC_M = 10.
# Next car at CP no sooner than this after our rear clears it (model times: its earliest
# arrival vs our slow-launch rear clearance). Chosen on the logged ring traffic: a GO forced
# at each of 350 frames standing at the line before 9 entries of 5 drives, replayed with
# the measured launch, the follower's 22.9 km/h entry limit and the window's simulator
# speed (test_ring_entry_replay). Every frame with a margin >= 0.55 left the next ring car
# >= 0.66 s sim body to body (post-encroachment) and >= 3.4 m apart (0.43 s / 2.2 m if
# the simulator ran 5 % slower than estimated); below it 0.54 left 0.47 s / 2.2 m, 0.25
# left 0.08 s and 0.16 touched. 1.0 held follow-20261010T140051Z lap 1 5.4 s at 0.98.
NEXT_MARGIN_S = .55
# Speed bounds: good position fit (RingMotionTracker) / detector velocity only.
FIT_RATIO = 1.1
FALLBACK_RATIO = 1.3
# Any ring car may speed up: this acceleration up to this speed. Ring NPCs drive 20.7
# km/h (5.75 m/s) and leave the spawn point at ~2 m/s^2; converted with an RTF that
# froze lower while we stood (0.636 vs ~0.75) they read up to ~6.8 m/s.
RESUME_ACCEL_MPS2 = 2.
RESUME_TOP_MPS = 7.
STANDING_MPS = .3
# A ring car standing beside our merge while we stand at the line yields to us
# (MORAI NPCs stop for the ego): after this long it no longer holds us.
STANDOFF_S = 2.
STANDOFF_ARC_M = 10.
# Follow caps: an object is on our path within this lateral reach (its half
# width + ours + margin) and ahead once its rear is past our front - .3 m.
FOLLOW_LATERAL_MARGIN_M = .3
FOLLOW_MIN_GAP_M = 1.
FOLLOW_TIMES_S = (0., .4, .8, 1.2)


def following_speed_cap(gap_m, lead_mps, ego_mps, *, measured_body_clear=False):
    """A moving leader opens a gap immediately; its speed is part of the cap."""
    gap, lead, speed = float(gap_m), float(lead_mps), float(ego_mps)
    if (not all(math.isfinite(v) for v in (gap, lead, speed))
            or (gap <= .1 and not (measured_body_clear and lead > STANDING_MPS))):
        return 0.
    gap = max(0., gap)
    lead, speed = max(0., lead), max(0., speed)
    feedback = max(0., lead + .6 * (gap - (1. + .6 * speed)))
    # Include relative stopping distance and 0.3 s command/perception response.
    braking = max(0., math.sqrt(.75**2 + lead**2 + 5.*max(0., gap-.25))-.75)
    return min(feedback, braking)


def _bounded_launch_state(state, ego_mps):
    """An optional measured lower envelope cannot exceed current physical speed."""
    if not isinstance(state, dict):
        return None
    try:
        if any(isinstance(state[key], (bool, np.bool_)) for key in ('v_mps', 'dead_s', 'stamp')):
            return None
        speed, dead, stamp = (float(state[key]) for key in ('v_mps', 'dead_s', 'stamp'))
        actual = float(ego_mps)
        if (all(math.isfinite(value) for value in (speed, dead, stamp, actual))
                and 0. <= speed <= actual and 0. <= dead <= LAUNCH_SLOW[1] and stamp > 0.):
            return dict(v_mps=speed, dead_s=dead, stamp=stamp)
    except (KeyError, TypeError, ValueError, OverflowError):
        pass
    return None


def pose_launch_state(ego):
    """Bind the publisher's launch evidence to this exact measured ego sample.

    Absence or invalid evidence retains the ordinary rest/dead-time bound.
    The publisher owns how the measured lower envelope was established.
    """
    try:
        brake, accelerator = float(ego['brake']), float(ego['accel'])
        if (not all(math.isfinite(value) for value in (brake, accelerator))
                or not 0. <= brake <= .001 or accelerator <= 0.):
            return None
        state = _bounded_launch_state(ego.get('_junction_launch'), float(ego['speed_kmh'])/3.6)
        stamp = float(ego['stamp'])
        return state if state is not None and math.isfinite(stamp) and state['stamp'] == stamp else None
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def launch_time(distance, accel, dead, v0=0., vmax=math.inf):
    """Time (s) to cover ``distance`` from speed v0: dead time at v0, then accel up to vmax."""
    distance = float(distance)
    if distance <= 0.:
        return 0.
    v0, accel = max(0., float(v0)), max(.1, float(accel))
    vmax = max(v0, float(vmax), .1)
    covered = v0 * dead
    if covered >= distance:
        return distance / max(v0, 1e-6)
    rest, elapsed = distance - covered, float(dead)
    if v0 >= vmax - 1e-9:
        return elapsed + rest / vmax
    t_acc = (vmax - v0) / accel
    d_acc = v0 * t_acc + .5 * accel * t_acc * t_acc
    if d_acc >= rest:
        return elapsed + (math.sqrt(v0 * v0 + 2. * accel * rest) - v0) / accel
    return elapsed + t_acc + (rest - d_acc) / vmax


def profile_time(distance, accel, dead, v0=0., limit=None, start_s=0., brake=2.5, step=.05, horizon=20.):
    """Time to cover ``distance`` from speed v0: dead time at v0, then ``accel`` under
    the station speed limit ``limit(s)`` (braking at ``brake`` where it falls)."""
    if limit is None:
        return launch_time(distance, accel, dead, v0)
    distance = float(distance)
    if distance <= 0.:
        return 0.
    t, s, v = 0., 0., max(0., float(v0))
    while t < horizon:
        if t + 1e-9 >= dead:
            allowed = max(0., float(limit(start_s + s)))
            v_next = min(v + accel * step, allowed) if v <= allowed else max(allowed, v - brake * step)
        else:
            v_next = v
        ds = .5 * (v + v_next) * step
        if s + ds >= distance:
            return t + step * (distance - s) / max(ds, 1e-9)
        s, v, t = s + ds, v_next, t + step
    return horizon


def leader_row(leader, t):
    """A followed ring car t seconds on: an object row on its lane at its assumed speed."""
    lane_xy, lane_arc, arc0, speed, length, width, uid = leader
    arc = min(arc0 + speed * t, float(lane_arc[-1]))
    x, y = (float(np.interp(arc, lane_arc, lane_xy[:, i])) for i in (0, 1))
    j = int(np.clip(np.searchsorted(lane_arc, arc), 1, len(lane_arc) - 1))
    dx, dy = lane_xy[j] - lane_xy[j - 1]
    heading = math.atan2(dy, dx)
    vx, vy = speed * math.cos(heading), speed * math.sin(heading)
    return (x, y, max(length, width), 'lidar_car', 'npc', speed, length, width, math.degrees(heading),
            vx, vy, uid, 0., None, True)


def follow_launch_time(target_s, centre_s, v0, model, limit, leaders=(), frame=None, gain=LAUNCH_TRACK_GAIN,
                       brake=2.5, step=.1, horizon=15., ego_front_m=EGO_HALF_LENGTH_M):
    """Time until our centre reaches station ``target_s`` launching from ``centre_s``.

    Our launch ``model`` = (acceleration, dead time) tracks the cap with ``gain`` (1/s) as
    the follower does; the cap is the station ``limit`` and, with our path ``frame``, the
    same follow_cap the committed entry applies behind each followed leader, moved along
    its lane at its assumed speed (leader_row)."""
    accel, dead = model
    t, s, v = 0., float(centre_s), max(0., float(v0))
    if s >= target_s:
        return 0.
    while t < horizon:
        if t + 1e-9 >= dead:
            cap = max(0., float(limit(s)))
            if leaders and frame is not None:
                cap = min(cap, follow_cap([leader_row(leader, t) for leader in leaders], frame, s, v,
                                         ego_front_m=ego_front_m)[0])
            rate = min(accel, gain * (cap - v)) if cap >= v else max(-brake, gain * (cap - v))
            v_next = max(0., v + rate * step)
        else:
            v_next = v
        ds = .5 * (v + v_next) * step
        if s + ds >= target_s:
            return t + step * (target_s - s) / max(ds, 1e-9)
        s, v, t = s + ds, v_next, t + step
    return horizon


def resume_time(distance, v0=0., accel=RESUME_ACCEL_MPS2, top=RESUME_TOP_MPS):
    """Earliest arrival of a standing/slow car that drives on (no dead time)."""
    return launch_time(distance, accel, 0., v0, max(top, v0))


class PathFrame:
    """Our path in world coordinates with stations."""
    def __init__(self, world):
        world = np.asarray(world, dtype=float)[:, :2]
        step = np.r_[0., np.linalg.norm(np.diff(world, axis=0), axis=1)]
        keep = np.r_[True, step[1:] > 1e-3]
        self.xy = world[keep]
        self.s = np.cumsum(step[keep])
        tangent = np.gradient(self.xy, axis=0) if len(self.xy) > 1 else np.array([[1., 0.]])
        norm = np.linalg.norm(tangent, axis=1, keepdims=True)
        self.t = tangent / np.maximum(norm, 1e-9)

    def project(self, points, s_lo=-math.inf, s_hi=math.inf):
        """(station, distance, index) of each point's nearest path point within [s_lo, s_hi]."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        window = np.flatnonzero((self.s >= s_lo) & (self.s <= s_hi))
        if not len(window) or not len(points):
            n = len(points)
            return np.full(n, np.nan), np.full(n, np.inf), np.zeros(n, dtype=int)
        delta = points[:, None, :] - self.xy[None, window, :]
        distance = np.linalg.norm(delta, axis=2)
        nearest = np.argmin(distance, axis=1)
        index = window[nearest]
        return self.s[index], distance[np.arange(len(points)), nearest], index


def body_rear_clear_station(frame, boundary_s):
    """Prove an actual-body rear clearance; otherwise retain the legacy envelope.

    The rear axle follows the checked polyline within 0.1 m and the body heading
    within six degrees of either incident segment. Each corner must pass the
    boundary plane and project past every earlier PathFrame sample. Comparing
    squared distances to an ahead witness gives linear separating planes, so
    checking original segment endpoints also covers every point between them.
    """
    fallback = float(boundary_s) + EGO_HALF_LENGTH_M
    try:
        xy, stations = np.asarray(frame.xy, float), np.asarray(frame.s, float)
        boundary = float(boundary_s)
        steps = np.diff(stations)
        if (xy.shape != (len(stations), 2) or len(stations) < 3
                or not np.isfinite(xy).all() or not np.isfinite(stations).all()
                or not math.isfinite(boundary) or boundary < stations[0]
                or fallback > stations[-1] or np.any(steps <= 0.)
                or np.max(steps) > .75):
            return fallback
        directions = np.diff(xy, axis=0) / steps[:, None]
        if not np.allclose(np.linalg.norm(directions, axis=1), 1., atol=1e-6):
            return fallback
        first = max(0, int(np.searchsorted(stations, boundary, side='left'))-1)
        last = min(len(directions)-1, int(np.searchsorted(stations, fallback, side='right')))
        headings = np.unwrap(np.arctan2(directions[:, 1], directions[:, 0]))[first:last+1]
        low, high = float(np.min(headings))-math.radians(6.), float(np.max(headings))+math.radians(6.)
        if high-low >= math.pi:
            return fallback
        point = np.array([np.interp(boundary, stations, xy[:, axis]) for axis in (0, 1)])
        witness_index = int(np.searchsorted(stations, boundary, side='left'))
        witness = xy[witness_index]
        behind = xy[:witness_index]
        delta = witness-behind
        length = np.linalg.norm(delta, axis=1)
        if np.any(length < 1e-6):
            return fallback  # a self-crossing has no unambiguous station proof
        normals = delta / length[:, None]
        offsets = np.sum(.5*(witness+behind)*normals, axis=1)
        incident = directions[first:min(first+2, len(directions))]
        normals = np.vstack((normals, incident))
        offsets = np.r_[offsets, incident@point]
        angles = np.arctan2(normals[:, 1], normals[:, 0])
        lo, hi = low-angles, high-angles
        support = np.full(len(normals), -math.inf)
        for along, across in ((EGO_FRONT_M, EGO_HALF_WIDTH_M),
                              (EGO_FRONT_M, -EGO_HALF_WIDTH_M),
                              (-EGO_REAR_M, EGO_HALF_WIDTH_M),
                              (-EGO_REAR_M, -EGO_HALF_WIDTH_M)):
            values = np.maximum(-along*np.cos(lo)+across*np.sin(lo),
                                -along*np.cos(hi)+across*np.sin(hi))
            peak = math.atan2(across, -along)
            peak += 2.*math.pi*np.ceil((lo-peak)/(2.*math.pi))
            values = np.where(peak <= hi, math.hypot(along, across), values)
            support = np.maximum(support, values)
        # Tracking reserve plus a strict 2.5 cm station/sampling allowance.
        offsets += support+.1+.025
        samples = np.r_[boundary, stations[(stations > boundary) & (stations < fallback)], fallback]
        positions = np.column_stack([np.interp(samples, stations, xy[:, axis]) for axis in (0, 1)])
        margins = positions@normals.T-offsets
        if np.any(margins[-1] < 0.):
            return fallback
        target = boundary
        for column in margins.T:
            bad = np.flatnonzero(column < 0.)
            if len(bad):
                index = int(bad[-1])
                fraction = -column[index]/(column[index+1]-column[index])
                target = max(target, samples[index]+fraction*(samples[index+1]-samples[index]))
        return min(fallback, float(np.nextafter(target, math.inf)))
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
        return fallback


def straight_lane(position, velocity, length=LANE_LOOKUP_M, step=.5):
    """Measured straight-line motion as a lane path (no map)."""
    speed = float(np.linalg.norm(velocity))
    if speed < STANDING_MPS:
        return []
    direction = np.asarray(velocity, dtype=float) / speed
    arc = np.arange(0., length + 1e-9, step)
    return [(np.asarray(position, dtype=float) + arc[:, None] * direction, arc)]


def lane_conflict(lane_xy, lane_arc, frame, s_lo, s_hi):
    """Contact / CP / exit of one lane path with our path (None without a CP)."""
    stations, distance, index = frame.project(lane_xy, s_lo, s_hi)
    if not len(distance) or not np.isfinite(distance).any():
        return None
    cp = np.flatnonzero(distance <= CONFLICT_POINT_M)
    if not len(cp):
        return None
    i_cp = int(cp[0])
    contact = np.flatnonzero(distance[:i_cp + 1] <= LANE_CONTACT_M)
    i_lane = int(contact[0]) if len(contact) else i_cp
    leaving = np.flatnonzero(distance[i_cp:] > LANE_CONTACT_M)
    i_exit = i_cp + int(leaving[0]) if len(leaving) else None
    if i_exit is not None and lane_arc[i_exit] - lane_arc[i_cp] > CROSSING_EXIT_M:
        i_exit = None       # shares our lane for a while (merge; the ring car leads us)
    # Crossing (lane leaves our corridor right after CP) or merge (shares our path).
    return dict(arc_lane=float(lane_arc[i_lane]), arc_cp=float(lane_arc[i_cp]),
                arc_exit=None if i_exit is None else float(lane_arc[i_exit]),
                s_lane=float(stations[i_lane]), s_cp=float(stations[i_cp]),
                start_distance=float(distance[0]), start_s=float(stations[0]))


def object_velocity(obj):
    """Velocity (sim units) of a planner object row; zeros when not measured."""
    if obj[3] == 'lidar_car':
        if len(obj) > 14 and obj[14] and len(obj) > 10:
            velocity = np.asarray(obj[9:11], dtype=float)
            if velocity.shape == (2,) and np.isfinite(velocity).all():
                return velocity, True
        return np.zeros(2), False
    if len(obj) > 10:
        velocity = np.asarray(obj[9:11], dtype=float)
        if velocity.shape == (2,) and np.isfinite(velocity).all():
            return velocity, True
    if len(obj) > 8 and float(obj[5]) > 0.:
        yaw = math.radians(float(obj[8]))
        return float(obj[5]) * np.array([math.cos(yaw), math.sin(yaw)]), True
    return np.zeros(2), False


def object_size(obj):
    length = max(.5, float(obj[6])) if len(obj) > 6 else max(.5, float(obj[2]))
    width = max(.5, float(obj[7])) if len(obj) > 7 else max(.5, float(obj[2]))
    return length, width


def follow_cap(objects, frame, centre_s, ego_mps, times=FOLLOW_TIMES_S, *, ego_front_m=EGO_HALF_LENGTH_M,
               current_ego=None):
    """Speed cap following whatever is on our path ahead now or merging ahead soon.

    Returns (cap m/s or inf, leader dict or None). Cars beside or behind our
    front are not leaders (a committed entry clears them by driving on)."""
    best, leader = math.inf, None
    ego_mps = max(0., float(ego_mps))
    lo, hi = centre_s - 5., centre_s + 45.
    for obj in objects:
        if len(obj) < 5 or obj[4] == 'pedestrian':
            continue
        velocity, measured = object_velocity(obj)
        length, width = object_size(obj)
        position = np.asarray(obj[:2], dtype=float)
        yaw = math.radians(float(obj[8])) if len(obj) > 8 else 0.
        for t in times:
            point = position + velocity * t
            station, distance, index = frame.project(point, lo, hi)
            station, distance, index = float(station[0]), float(distance[0]), int(index[0])
            if not math.isfinite(distance):
                break
            if distance > EGO_HALF_WIDTH_M + width / 2. + FOLLOW_LATERAL_MARGIN_M:
                continue
            tangent = frame.t[index]
            axis = np.array([math.cos(yaw), math.sin(yaw)])
            along_axis = abs(float(axis @ tangent))
            half = .5 * (length * along_axis + width * math.sqrt(max(0., 1. - along_axis ** 2)))
            along = station - centre_s
            if along - half < ego_front_m - .3:
                break        # beside or behind our front: not a leader
            gap = along - half - ego_front_m
            lead = max(0., float(velocity @ tangent)) if measured else 0.
            body_clear = None
            if t <= 0.:
                if (current_ego is not None and gap <= .1 and lead > STANDING_MPS
                        and obj[3] == 'lidar_car' and len(obj)>15 and obj[14] and not obj[15]):
                    from junction_vehicle_motion import _separation
                    separation = _separation(current_ego, obj, position)
                    if math.isfinite(separation) and separation > .1:
                        # The departing ring car is laterally clear, but its
                        # projection onto the curved entry can have zero rear
                        # gap. Follow its measured opening speed while the full
                        # current-body/raw and collision forecasts stay active.
                        body_clear = separation
                cap = following_speed_cap(gap, lead, ego_mps,
                                          measured_body_clear=body_clear is not None)
            else:
                # It reaches our path ahead in t: arrive there no sooner than its rear
                # plus FOLLOW_MIN_GAP_M, then follow it.
                room = max(0., gap - FOLLOW_MIN_GAP_M)
                cap = min(room / t, following_speed_cap(room, lead, ego_mps))
            if cap < best:
                best = cap
                leader = dict(object_id=obj[11] if len(obj) > 11 else None, name=obj[3],
                              gap_m=round(gap, 2), lead_mps=round(lead, 2), t=t, along=along)
                if body_clear is not None:
                    leader['measured_body_clear_m'] = body_clear
            break
    return best, leader


class RingCar:
    __slots__ = ('obj', 'uid', 'name', 'length', 'width', 'speed', 'upper', 'lower', 'kind',
                 'conflict', 'on_path_along', 'observed_flow')

    def __init__(self, obj, speed, ratio, kind, conflict, on_path_along=None, observed_flow=False):
        self.obj, self.uid, self.name = obj, (obj[11] if len(obj) > 11 else id(obj)), obj[3]
        self.length, self.width = object_size(obj)
        self.speed = float(speed)
        ratio = max(1., float(ratio))
        self.upper, self.lower = self.speed * ratio, self.speed / ratio
        self.kind, self.conflict, self.on_path_along = kind, conflict, on_path_along
        self.observed_flow = bool(observed_flow)

    def eta_cp(self):
        """Nominal arrival of its centre at our CP."""
        return self.conflict['arc_cp'] / max(self.speed, 1e-6)

    def earliest_cp(self):
        arc = self.conflict['arc_cp']
        if self.kind != 'moving':
            return resume_time(arc, 0.)
        if self.observed_flow:
            # Circulating scenario traffic maintains its measured speed. Keep
            # the source fit's speed uncertainty, without a hypothetical burst
            # of acceleration or an additional 30% inflation of a good fit.
            # Arrival belongs to the leading body edge, not its centre. A
            # constant-speed forecast has no acceleration conservatism to
            # accidentally cover this half-body travel time.
            front = .5*math.hypot(self.length, self.width)+.1
            return max(0., arc-front) / max(self.upper, 1e-6)
        return min(arc / max(self.upper, 1e-6), resume_time(arc, self.speed))

    def rear_clears_late(self):
        """Latest time its rear leaves the stretch our nose enters (simulator s)."""
        c = self.conflict
        arc = c['arc_exit'] if c['arc_exit'] is not None else c['arc_lane']
        return (arc + self.length / 2.) / max(self.lower, 1e-6)


def classify(objects, frame, centre_s, lane_paths, speed_ratio=None, unknown_ids=(), ego_pose=None,
             observed_flow_ids=()):
    """RingCar entries for every object whose lane reaches our CP (or is on our path).

    ``ego_pose`` (x, y, heading rad) excludes cars in our own lane behind/beside us."""
    speed_ratio = speed_ratio or {}
    unknown_ids = set(unknown_ids or ())
    observed_flow_ids = frozenset(observed_flow_ids)
    cars, unknown = [], []
    s_lo, s_hi = centre_s - CONFLICT_BEHIND_M, centre_s + CONFLICT_AHEAD_M
    for obj in objects:
        if len(obj) < 5 or obj[4] != 'npc':
            continue
        if ego_pose is not None:
            dx, dy = float(obj[0]) - ego_pose[0], float(obj[1]) - ego_pose[1]
            forward = dx * math.cos(ego_pose[2]) + dy * math.sin(ego_pose[2])
            lateral = -dx * math.sin(ego_pose[2]) + dy * math.cos(ego_pose[2])
            if forward < EGO_HALF_LENGTH_M and abs(lateral) <= LANE_CONTACT_M:
                continue  # in our lane behind/beside us (a queue at the line): never a ring arrival
        velocity, measured = object_velocity(obj)
        speed = float(np.linalg.norm(velocity))
        uid = obj[11] if len(obj) > 11 else None
        is_unknown = (obj[3] == 'lidar_car' and not measured) or (uid is not None and uid in unknown_ids)
        heading = math.atan2(velocity[1], velocity[0]) if measured and speed >= STANDING_MPS else None
        paths = lane_paths(obj, heading, velocity if measured else None)
        best = None
        for lane_xy, lane_arc in paths:
            conflict = lane_conflict(lane_xy, lane_arc, frame, s_lo, s_hi)
            if conflict is not None and (best is None or conflict['arc_cp'] < best['arc_cp']):
                best = dict(conflict, lane=(lane_xy, lane_arc))
        if is_unknown:
            unknown.append((obj, best, bool(paths)))
            continue
        if best is None:
            continue
        kind = 'moving' if speed >= STANDING_MPS else 'standing'
        on_path = None
        if best['start_distance'] <= CONFLICT_POINT_M:
            on_path = best['start_s'] - centre_s
        flow = False
        if (uid in observed_flow_ids and kind == 'moving' and speed <= RESUME_TOP_MPS
                and best['arc_exit'] is None and len(obj)>15 and not obj[15]
                and object_size(obj)[0]<=6. and object_size(obj)[1]<=2.4):
            lane, arc = best['lane']
            directions = np.diff(np.asarray(lane)[np.asarray(arc)<=20.], axis=0)
            headings = np.unwrap(np.arctan2(directions[:, 1], directions[:, 0]))
            flow = len(headings)>2 and np.ptp(headings)>=math.radians(25.)
        cars.append(RingCar(obj, speed, speed_ratio.get(uid, FALLBACK_RATIO), kind, best, on_path,
                            observed_flow=flow))
    return cars, unknown


def entry_decision(cars, centre_s, ego_mps, vmax, standoff_ids=(), speed_limit=None, frame=None,
                   *, yield_only=False, body_geometry=False, launch_state=None):
    """GO / WAIT at the line. Returns a dict with the timing evidence.

    ``speed_limit(station)`` (absolute path stations) bounds our own launch (curves,
    entry cap); without it ``vmax``. A followed leader bounds it through the
    follow-time term below, not by capping our launch at its speed from the start
    (we close the initial gap first). ``launch_state`` is an optional current
    publisher-confirmed lower envelope; callers with an ego pose must first
    bind its stamp with ``pose_launch_state``. It affects only the slow rear
    model, leaving earliest nose arrival and crossing margins unchanged.
    """
    front_m = EGO_FRONT_M if yield_only else EGO_HALF_LENGTH_M
    front = centre_s + front_m
    standoff_ids = set(standoff_ids or ())
    if speed_limit is None:
        def speed_limit(station, cap=float(vmax)):
            return cap

    def launch(distance, model, start):
        return profile_time(distance, model[0], model[1], ego_mps, speed_limit, start)
    ahead, after = [], []
    for car in cars:
        c = car.conflict
        if car.on_path_along is not None:
            if car.on_path_along - car.length / 2. >= front_m - .3:
                ahead.append(car)          # already in our lane ahead: followed
            continue                        # beside/behind us on our path: not a ring arrival
        if car.uid in standoff_ids:
            continue
        # Moving entry must use its earliest nose arrival, including the actual
        # rear-axle body origin and no standstill launch dead time.
        nose_distance = max(0., c['s_lane']-front)
        # A lower curve/command cap cannot make our earliest arrival later:
        # pending throttle and brake latency may carry the current speed on.
        # Use the acceleration upper bound without forecasting deceleration.
        if yield_only:
            # Stable finite solution of d = v*t + .5*a*t², including v=0.
            # The capped launch helper's infinite vmax path contains 0*inf
            # at rest, which cannot authorize a passing car's rear clearance.
            initial = max(0., float(ego_mps))
            nose = (2.*nose_distance/(initial+math.sqrt(initial*initial+6.4*nose_distance))
                    if nose_distance > 0. else 0.)
        else:
            nose = launch(nose_distance, LAUNCH_NOMINAL, centre_s)
        if (car.kind == 'moving' and c['arc_cp'] <= LEAD_ARC_M
                and car.rear_clears_late() + LEAD_GAP_S <= nose):
            ahead.append(car)
        else:
            after.append(car)
    followed = [car for car in ahead if car.conflict['arc_exit'] is None and
                (car.on_path_along is None or car.kind == 'moving')]
    # Circulating constant-speed traffic retains its measured lower bound.
    # Other leaders also retain the original possible merging slowdown.
    leaders = [(lead.conflict['lane'][0], lead.conflict['lane'][1], 0.,
                lead.lower if lead.observed_flow else min(lead.lower, lead.speed / LEADER_SPEED_RATIO),
                lead.length, lead.width, lead.uid) for lead in followed if 'lane' in lead.conflict]
    rear_cache = {}
    state = _bounded_launch_state(launch_state, ego_mps) if yield_only else None
    rear_speed = state['v_mps'] if state is not None else (0. if yield_only else ego_mps)
    rear_model = (LAUNCH_SLOW[0], state['dead_s']) if state is not None else LAUNCH_SLOW
    checks, blocker = [], None
    for car in after:
        c = car.conflict
        # Our centre station when our rear has cleared its lane (merge: CP; crossing: far edge).
        boundary = c['s_cp'] + (LANE_CONTACT_M if c['arc_exit'] is not None else 0.)
        target = (body_rear_clear_station(frame, boundary) if yield_only and body_geometry
                  else boundary+EGO_HALF_LENGTH_M)
        key = target if yield_only and body_geometry else round(target, 1)
        if key not in rear_cache:
            # Rest/dead time remains the default, including pending braking.
            # A current publisher-confirmed ongoing launch may instead start
            # from its measured lower envelope and remaining actuator delay.
            rear_cache[key] = follow_launch_time(target, centre_s, rear_speed, rear_model, speed_limit,
                                                 leaders, frame, ego_front_m=front_m)
        rear = rear_cache[key]
        earliest = car.earliest_cp()
        margin = earliest - rear
        checks.append(dict(object_id=car.uid, name=car.name, kind=car.kind, arc_cp=round(c['arc_cp'], 1),
                           speed=round(car.speed, 2), earliest_s=round(earliest, 2), rear_clear_s=round(rear, 2),
                           arrival_model='constant_velocity' if car.observed_flow else 'resume',
                           margin_s=round(margin, 2), required_rear_travel_m=max(0., target-centre_s)))
        if margin < NEXT_MARGIN_S - 1e-9 and (blocker is None or margin < blocker[0]):
            blocker = (margin, car, rear, earliest)
    leader = max(followed, key=lambda car: car.eta_cp(), default=None)
    result = dict(action='WAIT' if blocker else 'GO', checks=checks,
                  ahead=[car.uid for car in ahead], leader=None if leader is None else leader.uid)
    if blocker is not None:
        margin, car, rear, earliest = blocker
        result.update(blocker=car.uid, blocker_name=car.name, margin_s=round(margin, 2),
                      rear_clear_s=round(rear, 2), earliest_s=round(earliest, 2), s_cp=car.conflict['s_cp'])
    return result


def unknown_relevant(unknown, centre_s, ego_xy):
    """Unknown-motion tracks that can matter for our entry (ring arc/ETA scope)."""
    relevant = []
    for obj, conflict, mapped in unknown:
        if conflict is not None:
            if conflict['arc_cp'] <= RING_SCOPE_ARC_M or resume_time(conflict['arc_cp'], 0.) <= RING_SCOPE_ETA_S:
                relevant.append(obj)
        elif (not mapped and ego_xy is not None and
              np.linalg.norm(np.asarray(obj[:2], float) - np.asarray(ego_xy, float)) <= UNKNOWN_RANGE_M):
            # No lane to judge it by: a nearby unmeasured car is not a gap.
            relevant.append(obj)
    return relevant


class RingMotionTracker:
    """Robust ring-car speed from the measured box positions over ~1.5 s.

    The detector's own velocity (median of 0.3 s slopes) read the same ring car
    at 0-11 m/s sim within 2 s while it turned at the ring node (true 5.75 m/s);
    a straight-line fit of its positions over WINDOW_S at the detector stamps
    is within a few percent (chord of a 6 m arc on the 18 m ring: -1 %).
    Units stay measured (wall) here; the planner converts once (sim_time).
    """
    WINDOW_S = 2.
    MIN_SPAN_S = .8
    MIN_SAMPLES = 3
    MAX_RESIDUAL_M = 1.
    FORGET_S = 3.

    def __init__(self):
        self.history = {}        # uid -> list[(stamp, x, y, reported speed)]
        self.ratio = {}          # uid -> upper/lower speed bound ratio (dimensionless)
        self.fitted = {}         # uid -> fitted measured speed (diagnostics)

    def reset(self):
        self.history.clear()
        self.ratio.clear()
        self.fitted.clear()

    def fit(self, samples):
        data = np.asarray(samples, dtype=float)
        if len(data) < self.MIN_SAMPLES or data[-1, 0] - data[0, 0] < self.MIN_SPAN_S - 1e-9:
            return None
        t = data[:, 0] - data[-1, 0]
        design = np.column_stack((t, np.ones(len(t))))
        try:
            solution, _, rank, _ = np.linalg.lstsq(design, data[:, 1:3], rcond=None)
        except np.linalg.LinAlgError:
            return None
        if rank < 2:
            return None
        velocity = solution[0]
        residual = data[:, 1:3] - design @ solution
        rms = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        if not np.isfinite(velocity).all() or rms > self.MAX_RESIDUAL_M or np.linalg.norm(velocity) > 45.:
            return None
        return velocity

    def update(self, rows, stamps):
        """rows: VehicleReceiver rows; stamps: {uid: detector stamp}. Returns rows with robust speed."""
        latest = max(stamps.values()) if stamps else None
        out = []
        for row in rows:
            uid = int(row[11]) if len(row) > 11 and row[11] is not None else None
            stamp = stamps.get(uid) if uid is not None else None
            predicted = len(row) > 15 and row[15] is not None and float(row[15]) > 0.
            if row[3] != 'lidar_car' or stamp is None or predicted:
                out.append(row)
                continue
            measured = len(row) > 14 and bool(row[14])
            reported = np.asarray(row[9:11], dtype=float)
            samples = self.history.setdefault(uid, [])
            if not samples or stamp > samples[-1][0] + 1e-6:
                samples.append((float(stamp), float(row[0]), float(row[1]),
                                float(np.linalg.norm(reported)) if measured else math.nan))
            while samples and samples[-1][0] - samples[0][0] > self.WINDOW_S + 1e-9:
                samples.pop(0)
            velocity = self.fit([sample[:3] for sample in samples])
            if not measured:
                self.ratio[uid] = FALLBACK_RATIO
                self.fitted.pop(uid, None)
                out.append(row)
                continue
            if velocity is None:
                # No usable fit: the median of the detector's own recent speeds (one
                # turning frame read 11 m/s sim for a 5.75 m/s car), wide bounds.
                speeds = [sample[3] for sample in samples if math.isfinite(sample[3])]
                speed = float(np.median(speeds)) if speeds else float(np.linalg.norm(reported))
                ratio = FALLBACK_RATIO
            else:
                speed, ratio = float(np.linalg.norm(velocity)), FIT_RATIO
            # Partial box motion can underestimate a circulating car while its
            # measured scalar velocity remains steady. Keep the recent median
            # inside the arrival-speed bound, including line-controller users.
            recent_speeds = [sample[3] for sample in samples
                if samples[-1][0]-sample[0]<=1.5 and math.isfinite(sample[3])
                and sample[3]>=STANDING_MPS]
            if speed>=STANDING_MPS and len(recent_speeds)>=3:
                ratio = max(ratio, float(np.median(recent_speeds))/speed)
            if np.linalg.norm(reported) > STANDING_MPS:
                direction = reported / np.linalg.norm(reported)
            elif velocity is not None and speed > 1e-9:
                direction = velocity / speed
            else:
                direction = np.zeros(2)
            vx, vy = (direction * speed).tolist()
            values = list(row)
            values[5], values[9], values[10] = float(np.hypot(vx, vy)), vx, vy
            self.ratio[uid] = ratio
            self.fitted[uid] = speed
            out.append(tuple(values))
        if latest is not None:
            for uid in [uid for uid, samples in self.history.items()
                        if not samples or latest - samples[-1][0] > self.FORGET_S]:
                self.history.pop(uid, None)
                self.ratio.pop(uid, None)
                self.fitted.pop(uid, None)
        return out
