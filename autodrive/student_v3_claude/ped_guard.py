"""Track pedestrians and choose a temporal gap ahead of or behind their crossing.

All distances use the rear axle as origin. Crossing caps include the ego speed
transition; stationary people beside the path remain owned by the runtime's
standing-person bypass rule, those standing in our corridor ahead are held for.
"""
from __future__ import annotations

import math
import os

import numpy as np

from motion_limits import LAUNCH_ACCEL_MPS2 as LAUNCH_ACCEL

from ego_body import EGO_FRONT_M, EGO_REAR_M   # rear axle -> front / rear bumper (one body for every check)
HALF_W = .946
PED_R = .3
STOP_CLEAR_M = float(os.environ.get("STUDENT_PED_STOP_CLEAR_M", "2.5"))   # person's walk line -> body when stopped
HORIZON_S = 6.
SNAP_DEG = 25.
STEP_S = .1
FACTORS = (.8, 1., 1.2)          # crossing-time uncertainty
CORRIDOR_HALF_M = HALF_W + PED_R + .3
BEFORE_GAP_S, COMMITTED_GAP_S, AFTER_GAP_S = .8, .3, .6
MIN_YIELD_MPS = 2.
HARD_LAG_S, HARD_DECEL = .4, 4.5
COMFORT_LAG_S, COMFORT_DECEL = .5, 3.
LAUNCH_DEAD_S = .6
PROVISIONAL_S = 1.5
REVERSE_MPS, REVERSE_DEAD_S = .55, .8
# A track that held still over a 1.5-2 s fit is standing while its last-second fit
# stays below this (measured m/s). Fusion's own 0.3 s velocity jittered to 0.3-0.6
# m/s on people standing at the kerb beside our body and made them "cross" into it
# (follow-20261010T113545Z 20:38:22-31, 162527Z 01:27:54, 01:32:41).
SETTLED_FIT_MPS = .45
# Our front bumper can only reach a person whose body is ahead of its plane. A
# person entirely behind it (beside or behind the body) who is not walking toward
# our path faster than this cannot be protected by braking: stopping beside them
# only parked us there (follow-20261010T162527Z 01:27:50-28:01, 0-4 km/h for 11 s).
# People closer than BESIDE_CLEARANCE_M to our side keep every check (contact range).
BESIDE_MARGIN_M = .1
BESIDE_TOWARD_MPS = .5
BESIDE_CLEARANCE_M = .6
# Already stopping for these people (the caller's waiting set) at a stop point this
# close, slow or at a speed the approach envelope would let us keep: hold it.
# Creeping the last 0.3-0.6 m at 1-3 km/h while the person crossed in front
# re-armed every other gate (follow-20261010T162527Z 01:27:23.9-25.5).
STOP_HOLD_M, HOLD_MPS = 1., .5
# After a near-stop command for this exact person, a small change in the walk
# line must not restart the approach. In the 2026-10-12 404 rosbag, 0.98-0.99 m
# stops alternated with 1.03-1.11 m approaches and caused two real stops. Use one
# existing person radius as the exit band; this is a holding policy, not a claim
# that all perception error is bounded by 0.3 m. Safe crossing gaps still release
# immediately, and a stop point beyond this band returns to the normal approach.
STOP_RESUME_M = STOP_HOLD_M + PED_R
# Already rolling: when no cap >= MIN_YIELD_MPS arrives after the crossing, a lower
# one down to ROLL_MIN_MPS that still does is a deceleration, not a stop.
ROLL_MIN_MPS = .5
# Released people (we stopped for them and they cleared our path) walking out of it:
# assume they keep their measured pace (no 0.8x slow walker) and a 0.3 s exit
# margin; a nominal conflict then slows us (ROLL_MIN_MPS) before it stops us.
# Walking toward the path centre again (re-entry) gets the full rules.
RELEASED_FACTORS = (1., 1.2)
RELEASED_AFTER_GAP_S = .3
# Standing people (and walkers who froze) inside our corridor ahead of the front
# bumper are held for like a crossing: approach to STOP_CLEAR_M before them, hold
# there, go only once they have left the corridor. They used to be left to the
# older 4 s "pass around" rules: a walker who froze 1.4 m ahead of our left front
# was crept into at 1.5-2 km/h until the raw contact guard held the bumper
# 0.2-0.35 m from them for the rest of the run (follow-20261011T044808Z 13:50:31-42).
STAND_AHEAD_M = 30.
# The body leaves from where it is: a path whose start lies more than ANCHOR_MIN_M
# beside the rear axle (ROUTE BASE snapped back to the route after a bypass, 1.6 m
# right of the car) is judged as joined from the pose over ANCHOR_MERGE_M. Judged on
# the route itself, the person in front of our left front read 2.5 m beside "the
# path" (follow-20261011T044808Z 13:50:40.6-42.4).
ANCHOR_MIN_M = .3
ANCHOR_MERGE_M = 10.
STAND_MARGIN_M = CORRIDOR_HALF_M - HALF_W - PED_R    # 0.3 m: the crossing corridor's side margin
STAND_NEAR_M = 2.


class PedTracker:
    """Per-person position history -> velocity (simulator units); also the real-time factor."""

    def __init__(self, keep_s=2.5):
        self.keep_s = keep_s
        self.hist = {}          # key -> list[(stamp, x, y)]
        self.first = {}
        self.info = {}          # key -> dict(state, speed, age)
        self.motion = {}        # recent reliable velocity through fused-speed dropouts
        self.rtf = 1.
        self._rtf_valid = False
        self._ego = []

    def update_rtf(self, ego, now):
        """Distance the car really moved per wall second / reported speed (MORAI slower than real time)."""
        v = float(ego.get("speed_kmh", 0.)) / 3.6
        self._ego.append((now, float(ego["x"]), float(ego["y"]), v))
        self._ego = [e for e in self._ego if now - e[0] <= 3.]
        if v < 2.5 or len(self._ego) < 4:
            return self.rtf
        e = np.asarray(self._ego)
        if e[-1, 0] - e[0, 0] < 1. or e[:, 3].min() < 2.5:
            return self.rtf
        moved = float(np.sum(np.hypot(np.diff(e[:, 1]), np.diff(e[:, 2]))))
        expected = float(np.sum(.5 * (e[1:, 3] + e[:-1, 3]) * np.diff(e[:, 0])))
        if expected > 3.:
            ratio = float(np.clip(moved / expected, .3, 1.2))
            self.rtf = ratio if not self._rtf_valid else .7 * self.rtf + .3 * ratio
            self._rtf_valid = True
        return self.rtf

    @staticmethod
    def _fit(samples):
        a = np.asarray(samples, float)
        if len(a) < 4 or a[-1, 0] - a[0, 0] < .5:
            return None
        t = a[:, 0] - a[-1, 0]
        A = np.c_[t, np.ones(len(t))]
        cx, *_ = np.linalg.lstsq(A, a[:, 1], rcond=None)
        cy, *_ = np.linalg.lstsq(A, a[:, 2], rcond=None)
        res = np.hypot(a[:, 1] - A @ cx, a[:, 2] - A @ cy)
        rms = float(np.sqrt(np.mean(res ** 2)))
        vx, vy = float(cx[0]), float(cy[0])
        span = float(a[-1, 0] - a[0, 0])
        return vx, vy, math.hypot(vx, vy) * span, rms, span

    def _key(self, o, now):
        if len(o) > 11 and o[11] is not None:
            key = ("ped", int(o[11]))
            if key in self.hist:
                return key
        else:
            key = None
        # New id (or none): inherit a recent track lost this frame within 1 m (re-identification).
        best = None
        for k, h in self.hist.items():
            if now - h[-1][0] > .8 or k in self._seen:
                continue
            d = math.hypot(float(o[0]) - h[-1][1], float(o[1]) - h[-1][2])
            if d < 1. and (best is None or d < best[0]):
                best = (d, k)
        if best is not None:
            if key is not None:
                self.hist[key] = self.hist.pop(best[1])
                self.first[key] = self.first.pop(best[1], now)
                if best[1] in self.motion:
                    self.motion[key] = self.motion.pop(best[1])
                return key
            return best[1]
        return key if key is not None else ("pos", round(float(o[0]), 1), round(float(o[1]), 1), now)

    def update(self, objects, stamp, now, extra=None):
        """Return objects with pedestrian velocities (idx 5, 9, 10) refined; others unchanged.
        extra: {(\"pedestrian\", uid): [(t, x, y), ...]} full-rate history from the ROS receiver."""
        self._seen = set()
        self._last_info = {}
        out = []
        for o in objects:
            if o[4] != "pedestrian":
                out.append(o)
                continue
            key = self._key(o, now)
            self._seen.add(key)
            h = self.hist.setdefault(key, [])
            self.first.setdefault(key, now)
            more = extra.get(("pedestrian", int(o[11]))) if (extra and len(o) > 11 and o[11] is not None) else None
            if more:
                known_t = {round(x[0], 3) for x in h}
                h.extend(x for x in more if round(x[0], 3) not in known_t and (not h or x[0] > h[-1][0] - self.keep_s))
                h.sort(key=lambda x: x[0])
            if not h or stamp - h[-1][0] >= .04:
                h.append((stamp, float(o[0]), float(o[1])))
            self.hist[key] = h = [s for s in h if stamp - s[0] <= self.keep_s]
            fused = (float(o[9]), float(o[10])) if len(o) > 10 else (0., 0.)
            fused_speed = float(o[5])
            best, standing = None, False
            held_still, last_second = False, None
            for window in (1., 2.):
                fit = self._fit([s for s in h if stamp - s[0] <= window + 1e-6])
                if fit is None:
                    continue
                vx, vy, disp, rms, span = fit
                speed = math.hypot(vx, vy)
                if window == 1.:
                    last_second = speed
                if speed >= .2 and disp >= max(.2, 2.5 * rms):
                    if best is None or speed > best[2]:
                        best = (vx, vy, speed)
                elif span >= .9 and disp < max(.15, 1.5 * rms):
                    standing = True
                    held_still = held_still or (window == 2. and span >= 1.5)
            # Held still for 1.5-2 s and no real start within the last second: the
            # fused / 1 s jitter is not motion. A walker starting at >= 1 m/s passes
            # SETTLED_FIT_MPS in the 1 s fit within ~0.35 s.
            settled = held_still and (last_second is None or last_second < SETTLED_FIT_MPS)
            if settled:
                best = None
            scale = 1. / max(self.rtf, .3)
            if best is not None:
                vx, vy = best[0] * scale, best[1] * scale
                state = "moving"
            elif standing and (fused_speed < .5 or settled):
                vx = vy = 0.
                state = "standing"
            elif fused_speed >= .25:
                vx, vy = fused[0] * scale, fused[1] * scale
                state = "fused"
            elif key in self.motion and 0. <= now - self.motion[key][0] <= 2.:
                _, vx, vy = self.motion[key]
                state = "fused"
            else:
                vx = vy = 0.
                state = "unknown"
            if state in ("moving", "fused") and (best is not None or fused_speed >= .25):
                self.motion[key] = (now, vx, vy)
            elif state == "standing":
                self.motion.pop(key, None)
            speed = math.hypot(vx, vy)
            self.info[key] = dict(state=state, speed=speed, age=now - self.first[key])
            o = list(o)
            while len(o) < 12:
                o.append(0.)
            o[5], o[9], o[10] = speed, vx, vy
            o[11] = o[11] if o[11] else hash(key) % 100000
            out.append(tuple(o))
            self._last_info[int(o[11])] = self.info[key]
        for k in [k for k, h in self.hist.items() if not h or stamp - h[-1][0] > self.keep_s]:
            self.hist.pop(k, None); self.first.pop(k, None); self.info.pop(k, None)
            self.motion.pop(k, None)
        return out

    def state(self, uid):
        return getattr(self, "_last_info", {}).get(int(uid), {"state": "unknown", "speed": 0., "age": 0.})


class StandingBypassWait:
    """Per-person simulator-time wait; permission to search, never to drive.

    Count fresh stationary observations while stopped for that person. Bounded
    surface/centroid jitter pauses the count and suspends permission; translation
    beyond that noise envelope resets it. A missing observation or clock/source
    gap resets it too. A replacement id at the same spot can inherit the wait.
    """
    WAIT_SIM_S = 7.
    REDUCED_WAIT_SIM_S = 20.
    STATIC_WAIT_SIM_S = 40.
    # Requested staged policy, in simulator seconds. A timer permits searching
    # only; the final rear-axle body/raw-scan checks still decide whether to go.
    FIRST_GAP_M = .8
    REDUCED_GAP_M = .6
    BYPASS_CAP_MPS = 5./3.6
    MAX_GAP_S = .5

    def __init__(self):
        self.tracks = {}

    def update(self, objects, ego, now, rtf, tracker, waiting, stamp=None):
        from sim_time import clip_rtf
        old, current = self.tracks, {}
        used = set()
        seen_ids = {int(o[11]) for o in objects if o[4]=='pedestrian' and len(o)>11 and o[11] is not None}
        ready = set()
        for obj in objects:
            if obj[4] != "pedestrian" or len(obj) <= 11 or obj[11] is None:
                continue
            uid = int(obj[11])
            xy = np.asarray(obj[:2], float)
            entry = old.get(uid)
            if entry is None:
                matches = [(float(np.linalg.norm(xy - e['xy'])), k, e)
                           for k, e in old.items() if k not in used and k not in seen_ids and
                           0. <= now - e['last'] <= self.MAX_GAP_S]
                if matches:
                    distance, prior_uid, prior = min(matches, key=lambda row: row[0])
                    if distance < .6:
                        entry = prior
                        used.add(prior_uid)
            used.add(uid)
            fresh = entry is not None and 0. <= now - entry['last'] <= self.MAX_GAP_S
            if (fresh and stamp is not None and entry.get('stamp') is not None and
                    not 0. <= stamp-entry['stamp'] <= self.MAX_GAP_S):
                fresh = False
            info = tracker.state(uid) if tracker is not None else {}
            still = (info.get('state') == 'standing' and abs(float(obj[5])) < .2 and
                     np.isfinite(xy).all())
            # A short position change revokes a bypass before the long velocity
            # fit has caught up with a walker restarting from standstill.
            displacement = float(np.linalg.norm(xy-entry['anchor'])) if fresh else math.inf
            moved = displacement > .2
            off_since = entry.get('off_since') if fresh and still and moved else None
            if fresh and still and moved and off_since is None:
                off_since = now
            resettled = fresh and still and moved and (entry['elapsed']==0. or now-off_since >= 1.5)
            # A small restart inside the old anchor's noise envelope must also
            # revoke the timeout when this update precedes live revalidation.
            restarted = (fresh and displacement <= .2 and
                         np.linalg.norm(entry['xy']-entry['anchor']) <= .2 and
                         np.linalg.norm(xy-entry['xy']) >= .15)
            if not fresh or displacement > .55 or resettled:
                entry = dict(last=now, xy=xy, anchor=xy, elapsed=0., ready=False)
            elif restarted:
                # Pause the person bypass on this fresh change. A timeout
                # classification needs a new forty-second proof, while small
                # noisy centroids must not erase every pre-timeout wait.
                reset = entry['elapsed'] >= self.STATIC_WAIT_SIM_S
                entry = dict(entry, last=now, xy=xy, ready=False, off_since=None,
                             anchor=xy if reset else entry['anchor'],
                             elapsed=0. if reset else entry['elapsed'])
            elif not still or moved:
                # The logged stander alternated two body-surface centroids
                # 0.41 m apart. Do not count that observation or authorize a
                # bypass, but do not discard all prior stationary evidence.
                # A real walk leaves this bounded envelope and starts a new wait.
                entry = dict(entry, last=now, xy=xy, ready=False, off_since=off_since)
            else:
                entry = dict(entry)
                stopped_for = float(ego.get('speed_kmh', 0.)) < .5 and (uid in waiting or entry['elapsed'] > 0.)
                if stopped_for:
                    dt = (stamp-entry['stamp'] if stamp is not None and entry.get('stamp') is not None
                          else now-entry['last'])
                    entry['elapsed'] += dt * clip_rtf(rtf)
                elif not entry['ready']:
                    entry['elapsed'] = 0.
                entry['ready'] = entry['ready'] or entry['elapsed'] >= self.WAIT_SIM_S
                entry.update(last=now, xy=xy, off_since=None)
            entry['stamp'] = stamp
            current[uid] = entry
            if entry['ready'] and still and not moved:
                ready.add(uid)
        self.tracks = current
        return ready

    def policy(self, obj, now):
        """Fresh permission for this exact body; restart revokes it immediately."""
        entry = self.tracks.get(int(obj[11])) if len(obj)>11 and obj[11] is not None else None
        if (obj[4] != 'pedestrian' or entry is None or not entry['ready'] or
                not 0. <= now-entry['last'] <= self.MAX_GAP_S or abs(float(obj[5])) >= .2 or
                np.linalg.norm(np.asarray(obj[:2])-entry['xy']) >= .15 or
                np.linalg.norm(np.asarray(obj[:2])-entry['anchor']) > .2):
            return None
        elapsed = entry['elapsed']
        return dict(gap_m=self.REDUCED_GAP_M if elapsed >= self.REDUCED_WAIT_SIM_S else self.FIRST_GAP_M,
                    cap_mps=self.BYPASS_CAP_MPS, static=elapsed >= self.STATIC_WAIT_SIM_S,
                    reason='standing_for_40_sim_s' if elapsed >= self.STATIC_WAIT_SIM_S else None,
                    waited_sim_s=elapsed)


def local_xy(x, y, ego):
    h = math.radians(float(ego["heading_deg"]))
    dx, dy = float(x) - float(ego["x"]), float(y) - float(ego["y"])
    return dx * math.cos(h) + dy * math.sin(h), -dx * math.sin(h) + dy * math.cos(h)


class PathFrame:
    def __init__(self, path_local):
        p = np.asarray(path_local, float)[:, :2]
        keep = np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-3]
        p = p[keep]
        self.p = p
        t = np.gradient(p, axis=0)
        self.t = t / np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-6)
        self.n = np.column_stack((-self.t[:, 1], self.t[:, 0]))
        self.s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]

    def project(self, pts):
        pts = np.asarray(pts, float).reshape(-1, 2)
        j = np.argmin(((pts[:, None, :] - self.p[None, :, :]) ** 2).sum(-1), axis=1)
        d = pts - self.p[j]
        return self.s[j] + (d * self.t[j]).sum(1), (d * self.n[j]).sum(1), j


def anchor_path(path_local, min_offset_m=ANCHOR_MIN_M, merge_m=ANCHOR_MERGE_M):
    """The path as the body drives it from the pose (rear-axle origin).

    Unchanged when the origin lies within min_offset_m of the path. Otherwise the
    path is shifted sideways onto the origin there and rejoins its own line
    smoothly over merge_m of arc length ahead of it."""
    path = np.asarray(path_local, float)[:, :2]
    frame = PathFrame(path)
    if len(frame.p) < 3:
        return path
    s0, offset, _ = frame.project([[0., 0.]])
    offset = float(offset[0])
    if abs(offset) <= min_offset_m:
        return path
    u = np.clip((frame.s - float(s0[0])) / merge_m, 0., 1.)
    weight = 1. - u * u * (3. - 2. * u)
    return frame.p + frame.n * (offset * weight)[:, None]


def snap_to_road(vf, vl, tangent):
    """People walk along the road or straight across it (MORAI scenario walkers). Within
    SNAP_DEG of either, use that direction: a 10 deg error in a 1-2 s position fit put a
    person crossing 2 m ahead of our bumper into it 8 s later and backed us off (sim)."""
    speed = math.hypot(vf, vl)
    if speed < 1e-6:
        return vf, vl
    tx, ty = float(tangent[0]), float(tangent[1])
    par, perp = vf * tx + vl * ty, -vf * ty + vl * tx
    ang = abs(math.degrees(math.atan2(perp, par)))
    if ang < SNAP_DEG or ang > 180. - SNAP_DEG:
        sign = math.copysign(speed, par)
        return sign * tx, sign * ty
    if abs(ang - 90.) < SNAP_DEG:
        sign = math.copysign(speed, perp)
        return -sign * ty, sign * tx
    return vf, vl


def _clearance(along, lat, s_car):
    """Person (along, lat) arrays [..., T] vs the body at rear-axle station s_car [T]."""
    dx = np.maximum(0., np.maximum((s_car - EGO_REAR_M) - along, along - (s_car + EGO_FRONT_M)))
    dy = np.maximum(0., np.abs(lat) - HALF_W)
    return np.hypot(dx, dy) - PED_R


def _stop_envelope_cap(remaining, *, lag_s=COMFORT_LAG_S):
    """Recomputed approach cap; zero only at the requested stop point."""
    remaining = max(0., remaining)
    # Reserve the full controller response even near the stop point. Shortening
    # this lag as distance shrinks would authorize a speed that the next frame
    # cannot brake from, turning a controlled approach into an early hard stop.
    lag = lag_s
    return math.sqrt((COMFORT_DECEL * lag) ** 2 + 2. * COMFORT_DECEL * remaining) - COMFORT_DECEL * lag


def _profile(kind, v, t, *, v_target=None, stop_m=None, reverse_m=None):
    """Rear-axle travel (m) along the path at times t for a speed plan."""
    if kind == "keep":
        return v * t
    if kind == "go":
        vt = max(v, v_target or v)
        dead = LAUNCH_DEAD_S if v < 1. else 0.
        tt = np.maximum(0., t - dead)
        t_up = (vt - v) / LAUNCH_ACCEL
        return v * np.minimum(t, dead) + np.where(tt < t_up, v * tt + .5 * LAUNCH_ACCEL * tt ** 2,
                                                    v * t_up + .5 * LAUNCH_ACCEL * t_up ** 2 + vt * (tt - t_up))
    if kind == "speed":
        target = max(0., float(v_target))
        if target >= v:
            return _profile("go", v, t, v_target=target)
        lag = np.minimum(t, COMFORT_LAG_S)
        tt = np.maximum(0., t - COMFORT_LAG_S)
        down = (v - target) / COMFORT_DECEL
        tb = np.minimum(tt, down)
        return v * lag + v * tb - .5 * COMFORT_DECEL * tb ** 2 + target * np.maximum(0., tt - down)
    if kind == "hard":
        lag = np.minimum(t, HARD_LAG_S)
        tb = np.minimum(np.maximum(0., t - HARD_LAG_S), v / HARD_DECEL)
        return v * lag + v * tb - .5 * HARD_DECEL * tb ** 2
    if kind == "stop_envelope":
        # Forecast the per-frame command, rather than cruise forever at today's
        # cap. Keep the same initial brake response and acceleration limits as
        # "speed". Do not clamp travel to stop_m: a late observation can exceed
        # the stop point even under the maximum comfortable deceleration.
        steps = max(1, int(math.ceil(float(t[-1]) / .01)))
        clock = np.linspace(0., float(t[-1]), steps + 1)
        travel = np.zeros(steps + 1)
        initial, speed = max(0., v), max(0., v)
        ceiling = max(0., float(v_target))
        dead = LAUNCH_DEAD_S if initial < 1. else 0.
        for i, dt in enumerate(np.diff(clock)):
            # Evaluate the envelope at the end of this trapezoidal integration
            # step. Using its start value leaves speed above the shrinking cap
            # by one tick and spuriously breaks stop feasibility on replanning.
            target = min(ceiling, _stop_envelope_cap(stop_m - travel[i] - .5 * speed * dt,
                                                   lag_s=COMFORT_LAG_S + .5 * dt))
            if clock[i] < COMFORT_LAG_S:
                target = initial
            delta = min(LAUNCH_ACCEL * dt, max(-COMFORT_DECEL * dt, target - speed))
            if clock[i] < dead:
                delta = min(0., delta)
            travel[i + 1] = travel[i] + (speed + .5 * delta) * dt
            speed += delta
        return np.interp(t, clock, travel)
    if kind == "stopat":
        d = max(0., stop_m)
        lag = min(COMFORT_LAG_S, .2 * d / max(v, 1e-3))
        rest = max(0., d - v * lag)
        a = v * v / (2. * rest) if rest > 0. else HARD_DECEL
        tl = np.minimum(t, lag)
        tb = np.minimum(np.maximum(0., t - lag), v / max(a, 1e-3))
        return v * tl + v * tb - .5 * a * tb ** 2
    if kind == "stand":
        return np.zeros_like(t)
    if kind == "reverse":
        tt = np.maximum(0., t - REVERSE_DEAD_S)
        return -np.minimum(REVERSE_MPS * tt, reverse_m)
    raise ValueError(kind)


def _first_reach(t, difference):
    """Interpolated first zero crossing; infinity means the body has not arrived."""
    hits = np.flatnonzero(difference >= 0.)
    if not len(hits):
        return math.inf
    j = int(hits[0])
    if j == 0:
        return 0.
    u = -difference[j - 1] / max(difference[j] - difference[j - 1], 1e-9)
    return float(t[j - 1] + u * (t[j] - t[j - 1]))


def _crossing_interval(t, lat):
    """Times when a predicted walk enters/leaves the inflated body corridor."""
    inside = CORRIDOR_HALF_M - np.abs(lat)
    hits = np.flatnonzero(inside >= 0.)
    if not len(hits):
        return None
    last = int(hits[-1])
    enter = _first_reach(t, inside)
    if last == len(t) - 1:
        leave = math.inf
    else:
        u = inside[last] / max(inside[last] - inside[last + 1], 1e-9)
        leave = float(t[last] + u * (t[last + 1] - t[last]))
    return enter, leave


def _path_poses(frame, s0, travel):
    """Rear-axle poses (x, y, yaw) after these travels along the path from the pose.
    The first is the car as it stands (origin, heading 0)."""
    s = np.clip(s0 + np.asarray(travel, float), frame.s[0], frame.s[-1])
    poses = np.column_stack((np.interp(s, frame.s, frame.p[:, 0]), np.interp(s, frame.s, frame.p[:, 1]),
                             np.arctan2(np.interp(s, frame.s, frame.t[:, 1]), np.interp(s, frame.s, frame.t[:, 0]))))
    poses[0] = 0.
    return poses


def _sweep_gaps(poses, f, l):
    """Gap (m) of a person at (f, l) to the true body at each rear-axle pose (x, y, yaw)."""
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    dx, dy = f - poses[:, 0], l - poses[:, 1]
    ahead, across = dx * c + dy * s, -dx * s + dy * c       # person in each body frame (body_gap_along)
    return np.hypot(np.maximum(np.maximum(-EGO_REAR_M - ahead, ahead - EGO_FRONT_M), 0.),
                    np.maximum(np.abs(across) - HALF_W, 0.)) - PED_R


def _stand_stop_m(frame, s0, f, l, along0, lat0):
    """Rear-axle travel to the stop point for a standing person, or None if not in our way.

    In our way: inside the path corridor at their station, within STAND_MARGIN_M of
    the true body swept along the (pose-anchored) path - its front overhangs a bending
    path: the rejoin bent 0.7 m away from a person our left front was heading into -
    or of the body driven straight on for the next STAND_NEAR_M (the car crept straight
    at 1.5 km/h while the path pointed back to the route). The stop point leaves
    STOP_CLEAR_M from the front bumper to the person's centre (their walk line when
    they cross on), i.e. ~2.2 m to their body."""
    if abs(lat0) > CORRIDOR_HALF_M + 2.:
        return None
    stops = []
    if abs(lat0) <= CORRIDOR_HALF_M:
        stops.append(along0 - EGO_FRONT_M - STOP_CLEAR_M)
    travel = np.arange(0., max(0., along0 + EGO_REAR_M) + .1, .1)
    near = travel[travel <= STAND_NEAR_M]
    for poses, steps in ((_path_poses(frame, s0, travel), travel),
                         (np.column_stack((near, np.zeros_like(near), np.zeros_like(near))), near)):
        hit = np.flatnonzero(_sweep_gaps(poses, f, l) <= STAND_MARGIN_M)
        if len(hit):
            stops.append(float(steps[hit[0]]) - (STOP_CLEAR_M - PED_R - STAND_MARGIN_M))
    return max(0., min(stops)) if stops else None


def _stand_limit(standers, v_mps, holding, stop_hold_ids=()):
    """Approach / hold for people standing in our way ahead (plan's standers:
    (uid, name, rear-axle travel to their stop point))."""
    if not standers:
        return None
    ids = {item[0] for item in standers}
    stop_m = min(item[2] for item in standers)
    base = dict(ids=ids, name=", ".join(sorted({item[1] for item in standers})), along=stop_m)
    if stop_m <= .05:
        return dict(base, kind="stop", cap_mps=0., stopping=False,
                    reason="person standing in our path: hold %.1f m before them" % STOP_CLEAR_M)
    cap = _stop_envelope_cap(stop_m)
    held_near = any(uid in stop_hold_ids and uid in holding and distance <= STOP_RESUME_M
                    for uid, _, distance in standers)
    if ((stop_m <= STOP_HOLD_M and ids <= set(holding) or held_near)
            and (v_mps < HOLD_MPS or cap >= v_mps - .05)):
        return dict(base, kind="stop", cap_mps=0., stopping=False,
                    reason="person standing in our path: hold short of stop point (%.2f m)" % stop_m)
    v = max(0., v_mps)
    t = np.arange(0., max(3., 2. * v / COMFORT_DECEL + COMFORT_LAG_S + 2.) + STEP_S, STEP_S)
    travel = _profile("stop_envelope", v, t, stop_m=stop_m, v_target=math.inf)
    if float(travel.max()) > stop_m + .01:
        return dict(base, kind="stop", cap_mps=0., stopping=True,
                    reason="person standing in our path: emergency brake; stopping envelope exceeds stop point")
    return dict(base, kind="yield", cap_mps=cap, stopping=True,
                reason="person standing in our path: stop %.1f m before them (%.2f m to go)" % (STOP_CLEAR_M, stop_m))


def _with_standers(result, stand):
    """Combine plan's crossing decision with the standing-person limit (more restrictive wins).

    Standing people stay blocking (the caller keeps waiting for them, never releases
    them) while they limit us; they are not added to ``ids``, so the older position
    rules keep seeing them too."""
    if stand is None:
        return result
    merged = dict(result, blocking_ids=set(result.get("blocking_ids", ())) | stand["ids"],
                  conflict_ids=set(result.get("conflict_ids", ())) | stand["ids"])
    if result["kind"] == "stop":
        return merged
    cap = result.get("cap_mps")
    if stand["kind"] != "stop" and cap is not None and result["kind"] == "yield" and cap <= stand["cap_mps"]:
        return merged
    name = ", ".join(n for n in (stand["name"], result.get("name")) if n)
    limit = stand["cap_mps"] if cap is None else min(cap, stand["cap_mps"])
    return dict(merged, kind=stand["kind"], cap_mps=limit, along=stand["along"], name=name,
                stopping=stand["stopping"], reason=stand["reason"])


def plan(objects, ego, path_local, v_mps, zone_mps, tracker=None, committed=None, now=0.,
         anticipate_crossing=False, released=None, holding=None, release_motion=None,
         stop_hold_ids=None):
    """Use the largest feasible crossing cap, stopping only if neither gap exists.

    ``ids`` excludes decided tracks from the legacy pedestrian rules; stationary
    tracks are left to those rules so the existing bypass timer keeps working.
    A stationary person inside our corridor ahead of the front bumper is also held
    for here (blocking, not in ``ids``): approach to STOP_CLEAR_M before them and
    hold until they leave. A checked bypass around them puts them outside the
    corridor of the path it publishes. The path is judged from the pose
    (anchor_path). ``released`` holds people we already waited for and released
    (the caller's commit): they are predicted at their measured pace with a shorter exit margin,
    so only a nominal re-entry into our path can stop us again. ``blocking_ids``
    names the people a stop / stopping approach waits for; ``conflict_ids`` every
    person that still limits our speed. ``holding`` are the people we are
    already stopping for: near the stop point we hold rather than creep.
    ``release_motion`` maps a released person to their world velocity at the
    release: continuing that walk (or walking away from the path) keeps the
    relaxed prediction; turning back toward our path is judged in full.
    ``stop_hold_ids`` records a near-stop command per person. Only a still-blocking
    stopping approach within STOP_RESUME_M keeps it; checked temporal gaps release
    normally. The returned set is authoritative for the next observation.
    """
    committed = committed if committed is not None else {}
    released = released if released is not None else ()
    holding = holding if holding is not None else ()
    release_motion = release_motion if release_motion is not None else {}
    stop_hold_ids = set(stop_hold_ids or ())
    out = dict(kind="clear", cap_mps=None, ids=set(), pass_ids=set(), reason=None,
               along=None, reverse_m=None, clearance=None, name=None, timing=[],
               blocking_ids=set(), conflict_ids=set(), stop_hold_ids=set())
    if path_local is None or len(path_local) < 3:
        return out
    frame = PathFrame(anchor_path(path_local))
    s0 = float(frame.project([[0., 0.]])[0][0])
    # Cover slow arrivals as well as the usual 6-second immediate forecast.
    horizon = max(HORIZON_S, (45. + EGO_REAR_M) / MIN_YIELD_MPS + 8.)
    t = np.arange(0., horizon + STEP_S, STEP_S)
    h = math.radians(float(ego["heading_deg"]))
    c, s = math.cos(h), math.sin(h)
    people, unknown, approach, standers = [], [], [], []
    for o in objects:
        if o[4] != "pedestrian":
            continue
        f, l = local_xy(o[0], o[1], ego)
        if not (-EGO_REAR_M - 6. < f < 45. and abs(l) < 12.):
            continue
        uid = int(o[11]) if len(o) > 11 and o[11] is not None else id(o) % 100000
        info = tracker.state(uid) if tracker is not None else {
            "state": "fused" if float(o[5]) >= .25 else "unknown", "age": 9.}
        vx, vy = (float(o[9]), float(o[10])) if len(o) > 10 else (0., 0.)
        vf, vl = vx * c + vy * s, -vx * s + vy * c
        a0, l0, j0 = frame.project([[f, l]])
        along0, lat0 = float(a0[0]) - s0, float(l0[0])
        vf, vl = snap_to_road(vf, vl, frame.t[int(j0[0])])
        lateral_speed = float(np.array([vf, vl]) @ frame.n[int(j0[0])])
        if (anticipate_crossing and v_mps > 3. / 3.6
                and 0. < along0 < 25. and 1.6 <= abs(lat0) < 3.2
                and math.hypot(vf, vl) < .8 and np.sign(lat0) * lateral_speed <= .2):
            # Contest walkers can start only when the ego is close. A standing
            # track, or slight pre-crossing motion, cannot authorize cruising
            # past the point where the car could still stop. Keep a positive
            # passing pace beside the path; measured crossings own the stop.
            remaining = max(0., along0 - EGO_FRONT_M - STOP_CLEAR_M)
            cap = max(10. / 3.6, _stop_envelope_cap(remaining))
            approach.append((uid, str(o[3]), cap,
                             f + PED_R + BESIDE_MARGIN_M > EGO_FRONT_M))
        if info["state"] not in ("moving", "fused") or math.hypot(vf, vl) < .2:
            stop_at = (_stand_stop_m(frame, s0, f, l, along0, lat0)
                       if along0 + PED_R + BESIDE_MARGIN_M > EGO_FRONT_M and along0 < STAND_AHEAD_M else None)
            if stop_at is not None:
                # Standing (or frozen mid-crossing) in the way of our body ahead of
                # the front bumper: our front reaches them unless we stop.
                standers.append((uid, str(o[3]), stop_at))
                continue
            if info["state"] == "unknown" and info.get("age", 9.) < PROVISIONAL_S and 1.6 <= abs(lat0) < 3.2:
                out["ids"].add(uid)
                unknown.append(str(o[3]))
            continue
        # A shoulder/parallel walk is ignored only when its entire prediction
        # stays outside the path corridor, including bends farther ahead.
        out["ids"].add(uid)
        # Walk-in evidence in measured units (tracker rows are simulator units).
        toward = -math.copysign(1., lat0) * lateral_speed * min(1., max(.3, float(getattr(tracker, "rtf", 1.))))
        if (along0 + PED_R + BESIDE_MARGIN_M <= EGO_FRONT_M and toward < BESIDE_TOWARD_MPS
                and abs(lat0) - PED_R - HALF_W >= BESIDE_CLEARANCE_M):
            # Beside or behind our body and not walking into it: our front cannot
            # reach them and braking cannot keep them off our side. Decided.
            continue
        # The commit relaxes a released person who keeps walking the way they
        # left (or walks away from our path); turning back is judged in full.
        heading = release_motion.get(uid)
        continuing = heading is not None and vx * heading[0] + vy * heading[1] > 0.
        relaxed = uid in released and (toward < 0. or continuing)
        factors = RELEASED_FACTORS if relaxed else FACTORS
        walks, intervals = [], []
        for factor in factors:
            al, la, _ = frame.project(np.column_stack((f + vf * factor * t, l + vl * factor * t)))
            walks.append((al - s0, la))
            interval = _crossing_interval(t, la)
            if interval is not None:
                intervals.append(interval)
        if not intervals:
            continue
        # A rear walker with forward motion can enter our body corridor during
        # launch. Exclude only predictions that remain behind the rear bumper.
        if all(np.all(al < -EGO_REAR_M) for al, _ in walks):
            continue
        enter = min(x[0] for x in intervals)
        leave = max(x[1] for x in intervals)
        # The 1.4x walk is a final contact check only for close observations.
        fast = None
        if along0 <= 8.:
            al, la, _ = frame.project(np.column_stack((f + vf * 1.4 * t, l + vl * 1.4 * t)))
            fast = (al - s0, la)
        nominal_al, nominal_lat = walks[factors.index(1.)]
        crossing = np.abs(nominal_lat) <= CORRIDOR_HALF_M
        stop_line = float(nominal_al[crossing].min()) if crossing.any() else along0
        people.append(dict(uid=uid, name=str(o[3]), walks=walks, enter=enter,
                           exit=leave, along=along0, stop_line=stop_line, fast=fast,
                           after=RELEASED_AFTER_GAP_S if relaxed else AFTER_GAP_S, relaxed=relaxed))

    # Keep the existing temporal forecast, but do not make its soft approach
    # hold a mandatory braking cap after the actual front has passed someone.
    # The live checker still owns that person's possible departure beside us.
    ahead_caps = [item[2] for item in approach if item[3]]
    approach_cap = min(zone_mps, min(ahead_caps)) if ahead_caps else None
    if approach:
        zone_mps = min(zone_mps, min(item[2] for item in approach))
    stand = _stand_limit(standers, v_mps, holding, stop_hold_ids)
    if stand is not None:
        # The crossing decisions below execute at most this approach.
        zone_mps = min(zone_mps, stand["cap_mps"])

    def finish(result):
        result = _with_standers(result, stand)
        # Use each blocking person's own stop point. A nearby walker cannot
        # latch another person's distant approach in a combined decision.
        distances = {uid: distance for uid, _, distance in standers}
        distances.update({p['uid']: max(0., p['stop_line'] - EGO_FRONT_M - STOP_CLEAR_M)
                          for p in people})
        result = dict(result, stop_hold_ids={
            uid for uid in result.get('blocking_ids', ())
            if result['kind'] == 'stop' and uid in distances and
            distances[uid] <= (STOP_RESUME_M if uid in stop_hold_ids else STOP_HOLD_M)})
        # The caller may delegate an unknown-motion speed hold to its live
        # checker, but this stopping-readiness bound must survive delegation.
        # A temporal pass/yield for another walker also keeps this bound: those
        # decisions check both the current pace and the limited zone launch.
        if approach_cap is not None:
            cap = result.get("cap_mps")
            result = dict(result, approach_cap_mps=approach_cap,
                          cap_mps=approach_cap if cap is None else min(cap, approach_cap))
        return result

    def with_hold(result):
        """An undecided track limits every combined decision, not only empty ones."""
        if unknown and result["kind"] in ("clear", "pass"):
            result = dict(result, kind="hold", cap_mps=min(max(v_mps, 0.), zone_mps) if approach else max(v_mps, 0.),
                        name=", ".join(unknown),
                        reason="person beside the path, motion unknown: hold speed")
        if approach and result["kind"] in ("clear", "hold"):
            current_cap = result.get("cap_mps")
            return dict(result, kind="hold", cap_mps=min(zone_mps, zone_mps if current_cap is None else current_cap),
                        ids=set(result["ids"]) | {item[0] for item in approach},
                        name=", ".join(item[1] for item in approach),
                        reason="person beside obstacle mission: approach ready to stop")
        return result

    if not people:
        return finish(with_hold(out))

    def clearance_of(travel):
        walks = [walk for p in people for walk in p["walks"] + ([p["fast"]] if p["fast"] is not None else [])]
        return min(float(_clearance(*walk, travel).min()) for walk in walks)

    def clearance_for_cap(cap):
        # The publisher also applies the zone limit. A high approach cap cannot
        # authorize accelerating through a conflict when the zone keeps us slow.
        return min(clearance_of(_profile("speed", max(0., v_mps), t, v_target=target))
                   for target in (cap, min(cap, max(0., zone_mps))))

    def times(p, target):
        travel = _profile("speed", max(0., v_mps), t, v_target=target)
        # The front may come within STOP_CLEAR_M of a walk only after the person has
        # left our corridor. Arriving "after" them at the walk line itself released
        # a walker 3.9 m ahead of our bumper mid-crossing, and we closed to 1.4 m of
        # them (they froze in front of us; follow-20261011T044808Z 13:50:28.5-31.3).
        front = min(_first_reach(t, travel + EGO_FRONT_M + STOP_CLEAR_M - al) for al, _ in p["walks"])
        rear = max(_first_reach(t, travel - EGO_REAR_M - al) for al, _ in p["walks"])
        safe = p["fast"] is None or float(_clearance(*p["fast"], travel).min()) > 0.
        return front, rear, safe

    ahead, behind, already_behind = [], [], []
    # The zone target is executed even when it requires slowing down. An unknown
    # person's hold also limits acceleration, without cancelling a lower zone.
    # At a standstill the live checker owns the unknown person's hold (the caller
    # lifts that cap to the zone), so a crossing person is judged at the zone
    # launch: "clear at a crawl" there released us into a stop-go beside them.
    launch_target = max(0., min(v_mps, zone_mps) if unknown and v_mps >= HOLD_MPS else zone_mps)
    launch_travel = _profile("speed", max(0., v_mps), t, v_target=launch_target)
    for p in people:
        front, rear, safe = times(p, v_mps)
        # Releasing the cap executes the zone target, by acceleration or braking.
        # Both it and the current trajectory must preserve the chosen gap.
        launch_front, launch_rear, launch_safe = times(p, launch_target)
        launch_clearance = min(float(_clearance(*walk, launch_travel).min()) for walk in p["walks"])
        gap = COMMITTED_GAP_S if committed.get(p["uid"], 0.) > now else BEFORE_GAP_S
        out["timing"].append(dict(uid=p["uid"], t_enter=p["enter"], t_exit=p["exit"],
                                  t_arr=front, t_rear=rear))
        if (rear <= p["enter"] - gap and launch_rear <= p["enter"] - gap
                and safe and launch_safe and launch_clearance > 0.):
            ahead.append((p, gap))
        elif (front >= p["exit"] + p["after"] and launch_front >= p["exit"] + p["after"]
              and safe and launch_safe and launch_clearance > 0.):
            already_behind.append(p)
        else:
            behind.append(p)
    names = ", ".join(sorted({p["name"] for p in people}))
    if not behind:
        return finish(with_hold(dict(out, kind="pass" if ahead else "clear", name=names,
                                     pass_ids={p["uid"] for p, _ in ahead},
                                     reason="temporal gap: pass ahead" if ahead else None)))

    conflict = {p["uid"] for p in behind}
    out = dict(out, conflict_ids=conflict)

    def fits(target):
        for p in behind + already_behind:
            front, _, safe = times(p, target)
            if front < p["exit"] + p["after"] or not safe:
                return False
        return True

    def largest_fitting(lo, hi):
        for _ in range(24):
            mid = .5 * (lo + hi)
            if fits(mid):
                lo = mid
            else:
                hi = mid
        return lo

    ceiling = max(MIN_YIELD_MPS, v_mps, zone_mps) if not unknown else max(v_mps, 0.)
    if ceiling >= MIN_YIELD_MPS and fits(MIN_YIELD_MPS):
        lo = largest_fitting(MIN_YIELD_MPS, ceiling)
        ahead_ok = all(times(p, lo)[1] <= p["enter"] - gap and times(p, lo)[2] for p, gap in ahead)
        if ahead_ok and clearance_for_cap(lo) > 0.:
            return finish(dict(out, kind="yield", cap_mps=lo, name=names,
                               pass_ids={p["uid"] for p, _ in ahead},
                               along=min(p["along"] - EGO_FRONT_M for p in behind),
                               reason="temporal gap: arrive after crossing (cap %.2f m/s)" % lo))
    if (not unknown and released and v_mps >= ROLL_MIN_MPS and not ahead
            and all(p["relaxed"] for p in behind) and fits(ROLL_MIN_MPS)):
        # Committed and rolling after a stop we already made for these people:
        # slow down behind their walk instead of stopping again.
        lo = largest_fitting(ROLL_MIN_MPS, MIN_YIELD_MPS)
        if clearance_for_cap(lo) > 0.:
            return finish(dict(out, kind="yield", cap_mps=lo, name=names,
                               along=min(p["along"] - EGO_FRONT_M for p in behind),
                               reason="released crossing: slow down, arrive after it (cap %.2f m/s)" % lo))

    out = dict(out, blocking_ids=conflict)
    # Stop exactly 2.5 m before the crossing line; do not park early and creep.
    stop_m = max(0., min(p["stop_line"] for p in behind + [p for p, _ in ahead]) - EGO_FRONT_M - STOP_CLEAR_M)
    if stop_m <= .05:
        return finish(dict(out, kind="stop", cap_mps=0., along=stop_m, name=names,
                           reason="person crossing: wait at stop point"))
    held_near = any(p['uid'] in stop_hold_ids and p['uid'] in holding and
                    p['stop_line'] - EGO_FRONT_M - STOP_CLEAR_M <= STOP_RESUME_M
                    for p in behind)
    if ((stop_m <= STOP_HOLD_M and all(p["uid"] in holding for p in behind) or held_near)
            and (v_mps < HOLD_MPS or _stop_envelope_cap(stop_m) >= v_mps - .05)):
        # Already stopping for them, just short of the stop point: hold, rather
        # than crawl the last metre while they cross in front of us.
        return finish(dict(out, kind="stop", cap_mps=0., along=stop_m, name=names,
                           reason="person crossing: hold short of stop point (%.2f m)" % stop_m))
    # This envelope reaches zero only at stop_m. Keep the ideal stopat
    # deceleration diagnostic; the executed transition below reserves the full
    # response delay even when the ideal stopat would shorten it.
    lag = min(COMFORT_LAG_S, .2 * stop_m / max(v_mps, 1e-3))
    cap = _stop_envelope_cap(stop_m)
    if unknown:
        cap = min(cap, max(v_mps, 0.))
    required = v_mps * v_mps / (2. * max(1e-6, stop_m - v_mps * lag))
    # Both the uncapped approach and the actual zone/unknown limit must be
    # safe as the envelope shrinks. An ideal "stopat" may require more braking
    # than the controller can execute, so check the bounded transition too.
    ceilings = (max(0., v_mps), min(max(0., v_mps), max(0., zone_mps))) if unknown else (math.inf, max(0., zone_mps))
    approaches = [_profile("stop_envelope", max(0., v_mps), t, stop_m=stop_m, v_target=limit)
                  for limit in ceilings]
    selected_clearance = min(clearance_of(travel) for travel in approaches)
    exceeds_stop = any(float(travel.max()) > stop_m + .01 for travel in approaches)
    if selected_clearance <= 0. or exceeds_stop:
        return finish(dict(out, kind="stop", cap_mps=0., along=stop_m, name=names,
                           stopping=True, required_decel_mps2=required, clearance=selected_clearance,
                           reason="person crossing: emergency brake; stopping envelope " +
                                  ("predicts contact" if selected_clearance <= 0. else "exceeds stop point")))
    return finish(dict(out, kind="yield", cap_mps=cap, along=stop_m, name=names,
                       stopping=True, required_decel_mps2=required, clearance=selected_clearance,
                       reason="person crossing: stop %.1f m before walk (%.2f m to go%s)" %
                              (STOP_CLEAR_M, stop_m, "; braking limit exceeded" if required > HARD_DECEL else "")))


GHOST_MAX_MPS = 3.        # measured speed of a person-sized walker grown into a car box
GHOST_MIN_LENGTH_M = 3.   # only grown boxes (semantic_fusion grows a "vehicle" cluster to 4.6 m)


def person_vehicle_ghosts(objects, ego, person_scale=1.):
    """Indices of semantic-fusion car boxes that are a tracked person's own returns.

    semantic_fusion grows any camera-"vehicle" cluster into a 4.6 x 1.9 m box away
    from the ego. Part of a crossing person painted "vehicle" thus became a moving
    fusion_npc whose near face is the person, 2.1 m beyond them and moving with them:
    the gap planner's "predicted footprint time overlap" and the live "stopping
    envelope: fusion_npc" held us 1.3-1.6 s after the person had been released
    (follow-20261010T162527Z 01:27:24.1-25.0, 01:27:43.4-45.6; 113545Z 20:38:09-12).
    A real car cannot hold a person on its near face while moving at their pace.
    ``person_scale`` converts person velocities to the vehicle rows' units
    (PedTracker rows are simulator units: pass the real-time factor).
    """
    people = [o for o in objects if o[4] == "pedestrian" and len(o) > 10]
    if not people:
        return set()
    ego_xy = np.array([float(ego["x"]), float(ego["y"])])
    ghosts = set()
    for i, o in enumerate(objects):
        if (o[4] != "npc" or len(o) < 11 or not str(o[3]).startswith("fusion")
                or float(o[5]) > GHOST_MAX_MPS):
            continue
        length = max(float(o[6]), float(o[7]))
        if length < GHOST_MIN_LENGTH_M:
            continue
        centre = np.array([float(o[0]), float(o[1])])
        toward_ego = ego_xy - centre
        distance = float(np.linalg.norm(toward_ego))
        if distance < 1e-3:
            continue
        u = toward_ego / distance
        velocity = np.array([float(o[9]), float(o[10])])
        for p in people:
            d = np.array([float(p[0]), float(p[1])]) - centre
            along, across = float(d @ u), abs(float(d[0] * u[1] - d[1] * u[0]))
            person_velocity = np.array([float(p[9]), float(p[10])]) * float(person_scale)
            if (.3 <= along <= .5 * length + .3 and across <= .9
                    and float(np.linalg.norm(velocity - person_velocity)) <= 1.2):
                ghosts.add(i)
                break
    return ghosts


def without_person_ghosts(objects, ego, person_scale=1.):
    ghosts = person_vehicle_ghosts(objects, ego, person_scale)
    return [o for i, o in enumerate(objects) if i not in ghosts] if ghosts else list(objects)


class PersonGhostFilter:
    """without_person_ghosts that also keeps dropping an identified ghost id for
    ``hold_s`` while fusion misses the person for a frame (the box alone stopped
    us again: follow-20261010T162527Z 01:27:44.3)."""

    def __init__(self, hold_s=1.):
        self.hold_s, self.until = hold_s, {}

    def __call__(self, objects, ego, now, person_scale=1.):
        ghosts = person_vehicle_ghosts(objects, ego, person_scale)
        self.until = {uid: t for uid, t in self.until.items() if t > now}
        for i in ghosts:
            if len(objects[i]) > 11 and objects[i][11] is not None:
                self.until[objects[i][11]] = now + self.hold_s

        def remembered(o):
            return (o[4] == "npc" and len(o) > 11 and str(o[3]).startswith("fusion")
                    and float(o[5]) <= GHOST_MAX_MPS and o[11] in self.until)
        return [o for i, o in enumerate(objects) if i not in ghosts and not remembered(o)]


CONTACT_M = float(os.environ.get("STUDENT_CONTACT_GUARD_M", "0.35"))   # a person / object this close to the body


def contact_guard(points, people_local=()):
    """Auto run 268: a pedestrian fusion never listed stood against our right rear side
    (LiDAR body gap 0.05 m); creeping, tight passes and reverse escapes moved us 0-2 km/h
    back and forth along them and MORAI logged 14 separate contacts (210 s).

    Anything within CONTACT_M of the true body blocks the direction that would bring the
    body to it or drag the body along it: ahead of the front -> no forward; behind the
    rear -> no reverse; beside the body -> only the shorter way out (a person beside the
    rear lets us go forward, beside the front only back). Returns None or
    dict(block_forward, block_reverse, gap, where, back_m, ahead_m)."""
    rows = []
    if points is not None and len(points):
        p = np.asarray(points, dtype=float)[:, :2]
        rows.append(np.column_stack((p, np.zeros(len(p)))))
    for f, l in people_local:
        rows.append(np.array([[f, l, PED_R]]))
    if not rows:
        return None
    p = np.vstack(rows)
    dx = np.maximum(np.maximum(-EGO_REAR_M - p[:, 0], p[:, 0] - EGO_FRONT_M), 0.)
    dy = np.maximum(np.abs(p[:, 1]) - HALF_W, 0.)
    gap = np.hypot(dx, dy) - p[:, 2]
    inside = (dx == 0.) & (dy == 0.) & (p[:, 2] == 0.)    # own-body returns
    near = (gap <= CONTACT_M) & ~inside
    if not near.any():
        return None
    q = p[near]
    watch = (gap <= CONTACT_M + .5) & ~inside
    out = dict(block_forward=False, block_reverse=False, gap=round(float(gap[near].min()), 2), where=[],
               back_m=0., ahead_m=0., points=p[watch], gap0=float(gap[watch].min()))
    for x, y, _ in q:
        if x >= EGO_FRONT_M - .15:
            out["block_forward"] = True; out["where"].append("front")
        elif x <= -EGO_REAR_M + .15:
            out["block_reverse"] = True; out["where"].append("rear")
        else:
            # Beside the body: never back along it (reverse steering is loose; a 2 km/h
            # reverse past a person 0.05 m from our side touched them in the sim of 268).
            # Forward only on a path that does not swing the body toward them (sweep_ok).
            out["block_reverse"] = True
            out["ahead_m"] = max(out["ahead_m"], x + EGO_REAR_M + .4)
            out["where"].append("left" if y > 0 else "right")
    out["where"] = sorted(set(out["where"]))
    return out


def body_gap_along(points, poses):
    """Min gap of points (x, y, radius) to the true body at each pose (x, y, yaw) of the ego frame."""
    p = np.asarray(points, float)
    out = []
    for x, y, yaw in poses:
        c, s_ = math.cos(yaw), math.sin(yaw)
        dx0, dy0 = p[:, 0] - x, p[:, 1] - y
        lx, ly = dx0 * c + dy0 * s_, -dx0 * s_ + dy0 * c
        dx = np.maximum(np.maximum(-EGO_REAR_M - lx, lx - EGO_FRONT_M), 0.)
        dy = np.maximum(np.abs(ly) - HALF_W, 0.)
        out.append(float((np.hypot(dx, dy) - p[:, 2]).min()))
    return np.asarray(out)


def sweep_ok(guard, path_local, distance_m, reverse=False):
    """Moving `distance_m` along the path (or straight back) never brings the body closer
    than it is now to anything already within CONTACT_M (auto run 268: swerves and creeps
    along a person 0.05 m from our side)."""
    if not guard or not len(guard.get("points", ())):
        return True
    if reverse:
        poses = [(-d, 0., 0.) for d in np.arange(0., distance_m + 1e-6, .1)]
    else:
        path = np.asarray(path_local, float)[:, :2]
        ahead = path[path[:, 0] > -.5]
        if len(ahead) < 2:
            return True
        seg = np.linalg.norm(np.diff(ahead, axis=0), axis=1)
        s_ = np.r_[0., np.cumsum(seg)]
        poses = []
        for d in np.arange(0., distance_m + 1e-6, .1):
            j = int(np.clip(np.searchsorted(s_, d), 1, len(ahead) - 1))
            a, b = ahead[j - 1], ahead[j]
            u = (d - s_[j - 1]) / max(seg[j - 1], 1e-6)
            xy = a + (b - a) * min(max(u, 0.), 1.)
            poses.append((float(xy[0]), float(xy[1]), math.atan2(b[1] - a[1], b[0] - a[0])))
        # start from the car itself, not the path's first point
        poses[0] = (0., 0., 0.)
    gaps = body_gap_along(guard["points"], poses)
    floor = min(guard["gap0"], CONTACT_M)
    return bool(gaps.min() >= floor - .01)
