"""Lead-vehicle following (ACC) on the normal road.

Live run 264 (route idx ~400, the queue before the first signal junction): a slow,
stopping car ahead in our lane. The LiDAR route block shrank 26 -> 3 m in 4 s at
50-53 km/h, but outside the highway and the surveyed junctions the PointPillars
vehicle detector is off, the fused camera car came and went, and the lattice ignores
LiDAR returns outside the obstacle sections. Nothing braked until a fused car showed
up 5.5 m ahead: contact at 29.5 km/h (MORAI id 20002). Runs 244/254/166 stopped at
the same queue only 0.2 m short of it ("obstacle on control path", last-resort 3 m).

Every normal-road frame here gets one lead estimate, the nearest of
  * a vehicle object (fused camera car or LiDAR detector car) on the path we drive,
  * a car-like LiDAR return cluster in our body corridor along that path,
tracked over time for its speed, and a speed cap from the stopping distance:

    v * REACTION_S + v^2 / (2 BRAKE) <= gap - STANDSTILL_GAP + v_lead^2 / (2 LEAD_BRAKE)

A stopped (or not yet measured) lead is approached at a planned BRAKE deceleration
to STANDSTILL_GAP; a moving lead is followed at the gap where the same rule holds
(STANDSTILL_GAP + 0.5 v + 0.075 v^2 behind an equal-speed lead: 18.3 m at 40 km/h),
which is also the time-gap behaviour.

Units: ego speed, lead speed and the braking terms are simulator units. Object
velocities arrive converted (sim_time.to_sim_objects); LeadTracker's gap history
is timed by the wall clock, so its odometry and slopes use ``time_scale`` (RTF).

Pure numpy, no ROS; used by gui.Runtime.infer (normal-road branch).
"""
from __future__ import annotations

import math
import os

import numpy as np

from sim_time import clip_rtf

FRONT_M = 2.3175              # vehicle reference (body centre) to front bumper
BODY_HALF_M = .946
# Front bumper to the lead's rear at standstill. The stop overshoots this target by
# 0-1.1 m (logged rests at 4.5: 3.3-4.5 m, mostly 3.4-3.9 m, "wait behind it (gap
# 3.8 m)"); user 2026-10-10: 2.5-3 m, shorter but moderate. 3.5 m rests at ~2.4-3.5 m.
# EMERGENCY_GAP_M (1.5 m) and the full-stop check below are unchanged.
STANDSTILL_GAP_M = float(os.environ.get("STUDENT_LEAD_GAP_M", "3.5"))
BRAKE_MPS2 = float(os.environ.get("STUDENT_LEAD_BRAKE_MPS2", "2.5"))     # planned approach braking
LEAD_BRAKE_MPS2 = 4.0         # the lead's worst plausible braking (moving leads only)
REACTION_S = .5               # perception + decision + follower response
HORIZON_M = 60.               # 55 km/h needs ~59 m to stop behind a standing car
CORE_HALF_M = .8              # returns this close to the path centre always count
CORRIDOR_HALF_M = BODY_HALF_M + .35
FACE_SPAN_M = .4              # a cluster at least this wide across the path (a car's rear face)
# Beyond NEAR_M a car is hit by one VLP16 ring only, every ~0.17 m: >= 6 returns across
# 1 m when it is in our lane, >= 4 across 0.7 m when it overlaps our body by 0.35 m.
# Ground arcs where the road rises/banks ahead (S-curve, runs 259/264: 3-5 returns,
# 0.4-0.5 m) stay below that.
NEAR_M = 15.
FAR_RETURNS = 4
FAR_SPAN_M = .6
WINDOW_M = 1.5                # along-path extent of one cluster window
MIN_RETURNS = 3
TRACK_GATE_M = 3.             # association gate between frames (after prediction)
SPEED_WINDOW_S = 1.0
HOLD_S = .6                   # a lost lead is kept (predicted) this long: LiDAR/fusion flicker
MOVING_MPS = 1.0              # lead faster than this is moving (queue released)
STOPPED_MPS = .5


def path_coordinates(path_xy, points, horizon_m=HORIZON_M):
    """(s, l, index) of points along a vehicle-frame polyline: arc length from the first
    path point, signed lateral offset (left +) and the index into ``points``. Points far
    outside the path's bounding box are dropped; points beyond either path end get s = nan."""
    path = np.asarray(path_xy, dtype=float)[:, :2]
    pts = np.asarray(points, dtype=float).reshape(-1, 2) if points is not None else np.zeros((0, 2))
    empty = (np.zeros(0), np.zeros(0), np.zeros(0, dtype=int))
    if len(path) < 2 or not len(pts):
        return empty
    seg = np.diff(path, axis=0)
    length = np.linalg.norm(seg, axis=1)
    good = length > 1e-3
    path = np.vstack((path[:-1][good], path[-1:]))
    seg, length = np.diff(path, axis=0), np.linalg.norm(np.diff(path, axis=0), axis=1)
    if not len(seg):
        return empty
    arc = np.r_[0., np.cumsum(length)]
    keep = arc[:-1] <= horizon_m + 1.
    seg, length, start, arc0 = seg[keep], length[keep], path[:-1][keep], arc[:-1][keep]
    lo = np.minimum(start.min(axis=0), (start + seg).min(axis=0)) - CORRIDOR_HALF_M - .5
    hi = np.maximum(start.max(axis=0), (start + seg).max(axis=0)) + CORRIDOR_HALF_M + .5
    index = np.flatnonzero((pts[:, 0] >= lo[0]) & (pts[:, 0] <= hi[0]) & (pts[:, 1] >= lo[1]) & (pts[:, 1] <= hi[1]))
    pts = pts[index]
    if not len(pts):
        return empty
    # Dense raw tunnel scans made the full N x K x 2 temporary dominate
    # planning. Process cache-sized batches with the identical segment search;
    # retain every measured return and the original endpoint/tie behaviour.
    s, l = np.empty(len(pts)), np.empty(len(pts))
    batch_size = max(16, min(256, 32768 // len(seg)))
    for first in range(0, len(pts), batch_size):
        batch = pts[first:first+batch_size]
        rel = batch[:, None, :] - start[None, :, :]
        u = np.einsum("nkj,kj->nk", rel, seg) / (length * length)[None, :]
        t = np.clip(u, 0., 1.)
        projection = start[None]+t[..., None]*seg[None]
        dist2 = np.sum((batch[:, None, :]-projection)**2, axis=2)
        k = np.argmin(dist2, axis=1)
        n = np.arange(len(batch))
        station = arc0[k]+t[n, k]*length[k]
        lateral = (seg[k, 0]*rel[n, k, 1]-seg[k, 1]*rel[n, k, 0])/length[k]
        outside = ((k == 0) & (u[n, k] < 0.)) | ((k == len(seg)-1) & (u[n, k] > 1.))
        s[first:first+len(batch)] = np.where(outside, np.nan, station)
        l[first:first+len(batch)] = lateral
    return s, l, index


def path_heading_rad(path_xy, near_m=(0., 2.)):
    """Angle of the path chord between near_m[0] and near_m[1] m of arc (vehicle frame).
    On a straight road with an accurate pose this is ~0; a steady value is the
    localization heading bias (run 264: -3 deg, run 227: 2-5 deg)."""
    path = np.asarray(path_xy, dtype=float)[:, :2]
    if len(path) < 3:
        return 0.
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    if arc[-1] < near_m[1]:
        return 0.
    a = np.array([np.interp(near_m[0], arc, path[:, i]) for i in (0, 1)])
    b = np.array([np.interp(near_m[1], arc, path[:, i]) for i in (0, 1)])
    return math.atan2(b[1] - a[1], b[0] - a[0])


def body_aligned_path(path_xy, angle_rad):
    """The path shape re-anchored at the car: rotated by -angle_rad about its first point
    and moved to the vehicle origin. Where the localized heading is biased, the car
    physically drives this line, while the map path drawn in the vehicle frame swings
    off by angle x distance (run 264: 3 deg = 1.6 m at 30 m; the stopped car ahead fell
    outside the route corridor until 22 m)."""
    path = np.asarray(path_xy, dtype=float)[:, :2]
    if len(path) < 2:
        return path
    c, s = math.cos(-angle_rad), math.sin(-angle_rad)
    rel = path - path[0]
    return np.column_stack((c * rel[:, 0] - s * rel[:, 1], s * rel[:, 0] + c * rel[:, 1]))


def lidar_lead(path_xy, points, horizon_m=HORIZON_M, corridor_half_m=CORRIDOR_HALF_M):
    """Nearest car-like LiDAR cluster in our body corridor along the path, or None.

    A cluster blocks when it is at least FACE_SPAN_M wide across inside the corridor
    (a car's rear face) and reaches within CORE_HALF_M of the path centre. A thin line
    along the path beside the body (guardrail, kerb) does not.
    Distances are from the vehicle reference (body centre), like LidarCorridor.blocked."""
    s, l, _ = path_coordinates(path_xy, points, horizon_m)
    if not len(s):
        return None
    m = np.isfinite(s) & (s >= FRONT_M) & (s <= horizon_m) & (np.abs(l) <= corridor_half_m)
    if int(m.sum()) < 2:
        return None
    s, l = s[m], l[m]
    order = np.argsort(s)
    s, l = s[order], l[order]
    starts = np.unique(np.floor(s / .5) * .5)
    for s0 in starts:
        w = (s >= s0) & (s < s0 + WINDOW_M)
        lw = l[w]
        core = int(np.sum(np.abs(lw) <= CORE_HALF_M))
        span = float(np.ptp(lw)) if len(lw) else 0.
        # A car's rear face is wide across the path; a guardrail or kerb beside the body
        # (2-3 returns at one lateral offset, also where the corridor swings onto it at
        # 35-50 m in the S-curve, replay of run 264) is not.
        near = s0 < NEAR_M          # several rings hit a car there: fewer, narrower returns suffice
        if (len(lw) >= (MIN_RETURNS if near else FAR_RETURNS) and core >= 1
                and span >= (FACE_SPAN_M if near else FAR_SPAN_M)):
            first = float(s[w][0])
            return dict(dist_m=first, lat_m=float(np.median(lw)), n=int(len(lw)), span_m=round(span, 2),
                        core=core)
    return None


def object_leads(objects, ego, path_xy, horizon_m=HORIZON_M):
    """Vehicle objects ahead on the path: list of dict(dist_m, v_mps, valid, name, uid).

    dist_m is to the vehicle's rear (from our body centre). Same rule as GapPlanner's
    same-lane leader: within BODY_HALF + width/2 + 0.2 m of the path, heading within
    35 deg of it (or not moving)."""
    out = []
    if not objects:
        return out
    yaw = math.radians(float(ego["heading_deg"]))
    c, sn = math.cos(yaw), math.sin(yaw)
    cand = []
    for o in objects:
        if len(o) < 9 or o[4] != "npc":
            continue
        dx, dy = float(o[0]) - float(ego["x"]), float(o[1]) - float(ego["y"])
        f, lft = c * dx + sn * dy, -sn * dx + c * dy
        if not (0. < f < horizon_m + 5.):
            continue
        cand.append((o, f, lft))
    if not cand:
        return out
    s, l, index = path_coordinates(path_xy, [(f, lft) for _, f, lft in cand], horizon_m + 5.)
    if not len(s):
        return out
    path = np.asarray(path_xy, dtype=float)[:, :2]
    for j, i in enumerate(index):
        o, f, lft = cand[int(i)]
        if not np.isfinite(s[j]):
            continue
        length, width = max(.5, float(o[6])), max(.5, float(o[7]))
        if abs(l[j]) > BODY_HALF_M + width / 2 + .2:
            continue
        # path heading at the object
        k = int(np.argmin(np.linalg.norm(path - (f, lft), axis=1)))
        a, b = path[max(0, k - 1)], path[min(len(path) - 1, k + 1)]
        path_yaw = yaw + math.atan2(b[1] - a[1], b[0] - a[0])
        vx, vy = (float(o[9]), float(o[10])) if len(o) > 10 else (0., 0.)
        speed = math.hypot(vx, vy)
        valid = bool(o[14]) if len(o) > 14 else speed > 0.
        if speed > 1.:
            motion = math.atan2(vy, vx)
            if abs((motion - path_yaw + math.pi) % (2 * math.pi) - math.pi) > math.radians(35):
                continue          # crossing / oncoming: the gap planner times it
            along = speed * math.cos(motion - path_yaw)
        else:
            along = 0.
        rear = float(s[j]) - length / 2
        if rear < FRONT_M - .5:
            continue              # beside us, not ahead: the gap planner's business
        out.append(dict(dist_m=rear, v_mps=max(0., along), valid=valid,
                        name=str(o[3]), uid=o[11] if len(o) > 11 else None))
    out.sort(key=lambda d: d["dist_m"])
    return out


def follow_cap_mps(gap_m, lead_mps=0., reaction_s=REACTION_S, brake=BRAKE_MPS2, lead_brake=LEAD_BRAKE_MPS2,
                   standstill_m=STANDSTILL_GAP_M):
    """Highest speed that can still stop STANDSTILL_GAP behind the lead (bumper gap gap_m)."""
    if gap_m is None or not math.isfinite(gap_m):
        return float("inf")
    lead = max(0., float(lead_mps or 0.))
    budget = float(gap_m) - standstill_m + lead * lead / (2. * lead_brake)
    if budget <= 0.:
        return 0.
    bt = brake * reaction_s
    return max(0., -bt + math.sqrt(bt * bt + 2. * brake * budget))


class HeadingBias:
    """Low-passed path angle at the car (tau 2 s): the persistent localization heading
    bias, not the transient steering error. Ignored beyond 8 deg (not a bias)."""
    TAU_S = 2.
    MAX_RAD = math.radians(8.)

    def __init__(self):
        self.value, self.last_t = 0., None

    def update(self, now, angle_rad):
        if not math.isfinite(angle_rad) or abs(angle_rad) > self.MAX_RAD:
            angle_rad = 0.
        if self.last_t is None or not 0. <= now - self.last_t <= 1.:
            self.value = angle_rad
        else:
            k = min(1., (now - self.last_t) / self.TAU_S)
            self.value += k * (angle_rad - self.value)
        self.last_t = now
        return self.value


class LeadTracker:
    """One lead along our path: association, speed from the gap history, short hold.

    The lead's along-path position is our odometry plus the measured distance; its
    speed is the slope of that over SPEED_WINDOW_S (the lower of the full and the last
    half window: a braking lead is not overestimated). A vehicle object with a measured
    velocity gives the speed directly."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.odo = 0.
        self.last_t = None
        self.track = None
        self.last_scan = None

    @staticmethod
    def _slope(hist, window):
        recent = [h for h in hist if hist[-1][0] - h[0] <= window + 1e-6]
        if len(recent) < 3 or recent[-1][0] - recent[0][0] < .25:
            return None
        t = np.asarray([h[0] for h in recent]) - recent[-1][0]
        x = np.asarray([h[1] for h in recent])
        return float(np.polyfit(t, x, 1)[0])

    def _state(self, dist, track, held):
        return dict(dist_m=dist, gap_m=dist - FRONT_M, v_lead=track["v"], known=track["known"],
                    held=held, source=track["source"], confirmed=track["scans"] >= 2 or track["object"])

    def update(self, now, ego_mps, lidar=None, objects=(), scan_stamp=None, time_scale=1.):
        """lidar: lidar_lead() result or None; objects: object_leads() list; scan_stamp:
        LiDAR scan time (a repeated scan adds no speed sample). Returns
        dict(dist_m, gap_m, v_lead, known, held, source, confirmed) or None.

        ``now`` is the wall clock; ``time_scale`` the simulator real-time factor.
        Odometry integrates simulator speed over simulated time, and gap-history
        slopes (metres per wall second) become simulator speeds, as the object
        speeds already are. Hold and history windows stay sensor (wall) times."""
        scale = clip_rtf(time_scale)
        if self.last_t is not None and 0. <= now - self.last_t <= .5:
            self.odo += max(0., float(ego_mps)) * (now - self.last_t) * scale
        elif self.last_t is not None:
            self.track = None       # time jump: start over
        self.last_t = now
        meas = []
        if lidar is not None:
            meas.append(dict(dist_m=float(lidar["dist_m"]), v=None, source="lidar", object=False))
        for o in objects or ():
            meas.append(dict(dist_m=float(o["dist_m"]), v=float(o["v_mps"]) if o.get("valid") else None,
                             source=o.get("name", "object"), object=True))
        best = min(meas, key=lambda m: m["dist_m"]) if meas else None
        if best is not None and best["v"] is None:
            # A LiDAR cluster at a measured car takes the car's velocity.
            for m in meas:
                if m["v"] is not None and abs(m["dist_m"] - best["dist_m"]) < 3.:
                    best = dict(best, v=m["v"], source=best["source"] + "+" + m["source"], object=True)
                    break
        track = self.track
        if best is None:
            if track is None:
                return None
            # Lost this frame (LiDAR/fusion flicker): predicted from the last measurement.
            # A single-scan return is not held.
            if now - track["t"] > HOLD_S or not (track["scans"] >= 2 or track["object"]):
                self.track = None
                return None
            return self._state(track["station"] + track["v"] * (now - track["t"]) * scale - self.odo,
                               track, True)
        station = self.odo + best["dist_m"]
        if track is not None:
            predicted = track["station"] + track["v"] * (now - track["t"]) * scale
            if abs(station - predicted) > TRACK_GATE_M + .5 * abs(track["v"]):
                track = None
        new_scan = best["object"] or scan_stamp is None or scan_stamp != self.last_scan
        self.last_scan = scan_stamp
        if track is None:
            track = dict(hist=[], v=0., known=False, scans=0, object=False)
        if new_scan or not track["hist"]:
            track["hist"].append((now, station))
            track["scans"] += 1
        track["hist"] = [h for h in track["hist"] if now - h[0] <= SPEED_WINDOW_S]
        track["object"] = track["object"] or best["object"]
        if best["v"] is not None:
            track["v"], track["known"] = max(0., best["v"]), True
        else:
            slopes = [v / scale for v in (self._slope(track["hist"], SPEED_WINDOW_S),
                                          self._slope(track["hist"], SPEED_WINDOW_S / 2.)) if v is not None]
            if slopes:
                track["v"], track["known"] = max(0., min(slopes)), True
        track.update(station=station, t=now, source=best["source"])
        self.track = track
        return self._state(best["dist_m"], track, False)


def lead_cap(state, ego_mps=None):
    """Speed cap (m/s) for a LeadTracker state; an unmeasured lead counts as standing.

    A LiDAR cluster seen in one scan only is not acted on while there is still room to
    stop from the current speed at 5 m/s^2 after it is confirmed (one more scan):
    single-scan returns at 35-50 m (S-curve guardrail, run 264 replay) braked for
    nothing."""
    if state is None:
        return None
    if not state.get("confirmed", True) and ego_mps is not None:
        v = max(0., float(ego_mps))
        if state["gap_m"] > STANDSTILL_GAP_M + v * (REACTION_S + .3) + v * v / (2. * 5.):
            return None
    lead = state["v_lead"] if state["known"] else 0.
    return follow_cap_mps(state["gap_m"], lead)


EMERGENCY_DECEL_MPS2 = 3.5    # the follower's normal braking limit (sim: 3.5; a stop command brakes harder)
EMERGENCY_GAP_M = 1.5


def needs_emergency_stop(state, ego_mps):
    """True when even the follower's full normal braking cannot keep EMERGENCY_GAP_M to
    the lead (unmeasured lead: standing): send a full stop instead of a speed cap. A car
    first seen at 30 m from 55 km/h (VLP16 range for a car, run 264) needs ~5 m/s^2."""
    if state is None or not (state.get("confirmed", True) or state["gap_m"] < 15.):
        return False
    v = max(0., float(ego_mps))
    lead = state["v_lead"] if state["known"] else 0.
    closing = v - lead
    if closing <= .3:
        return False
    room = state["gap_m"] - EMERGENCY_GAP_M - v * .2
    if room <= .1:
        return True
    return closing * closing / (2. * room) > EMERGENCY_DECEL_MPS2
