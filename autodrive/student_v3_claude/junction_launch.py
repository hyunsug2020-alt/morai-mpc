"""Observed launch initial conditions, owned by successful command publication.

This memory grants neither GO nor commitment. Fresh traffic timing and complete
body checks must independently approve every command. Its speed is a cautious
observed estimate, not a physical bound on future braking or acceleration.
"""
from collections import deque
from dataclasses import dataclass
import math
import threading

import numpy as np

from ring_entry import LAUNCH_SLOW
from sim_time import RTF_MIN


def _geometry(points):
    xy = np.asarray(points, float)
    if xy.ndim != 2 or xy.shape[0] < 3 or xy.shape[1] < 2 or not np.isfinite(xy[:, :2]).all():
        return None
    xy = xy[:, :2].copy()
    xy = xy[np.r_[True, np.linalg.norm(np.diff(xy, axis=0), axis=1) > 1e-6]]
    if len(xy) < 3:
        return None
    length = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    arc = np.r_[0., np.cumsum(length)]
    return (xy, arc, np.diff(xy, axis=0)/length[:, None]) if arc[-1] >= 8. else None


def _project(points, geometry):
    xy, arc, tangent = geometry
    points = np.atleast_2d(points)
    delta = points[:, None, :]-xy[None, :-1]
    fraction = np.sum(delta*tangent[None], axis=2)/np.diff(arc)[None]
    clipped = np.clip(fraction, 0., 1.)
    closest = xy[None, :-1]+clipped[..., None]*np.diff(xy, axis=0)[None]
    distances = np.linalg.norm(points[:, None]-closest, axis=2)
    index = np.argmin(distances, axis=1)
    row = np.arange(len(points))
    station = arc[index]+clipped[row, index]*np.diff(arc)[index]
    inside = ~(((index == 0) & (fraction[row, index] < -1e-6)) |
               ((index == len(tangent)-1) & (fraction[row, index] > 1.+1e-6)))
    return station, distances[row, index], tangent[index], inside


def _compatible(first, second):
    """Allow a map-reference trim, checking the complete common extent both ways."""
    for reference, candidate in ((first, second), (second, first)):
        xy, arc, _ = candidate
        stations = np.unique(np.r_[arc, np.arange(0., arc[-1], .5)])
        points = np.column_stack([np.interp(stations, arc, xy[:, axis]) for axis in (0, 1)])
        _, _, own_tangent, _ = _project(points, candidate)
        mapped, distances, tangent, inside = _project(points, reference)
        selected = np.flatnonzero(inside)
        if (len(selected) < 3 or np.ptp(stations[selected]) < 8.-1e-6
                or np.any(distances[selected] > .1+1e-9)
                or np.any(np.sum(own_tangent[selected]*tangent[selected], axis=1) < math.cos(math.radians(6.)))
                or np.any(np.diff(mapped[selected]) < -.01)):
            return False
    return True


@dataclass(frozen=True)
class _Observation:
    stamp: float
    xy: np.ndarray
    station: float
    speed: float
    accel: float
    brake: float
    now: float


class JunctionLaunchMemory:
    """Publisher calls published/clear; predictors may only read model evidence."""
    def __init__(self):
        self._lock = threading.RLock()
        self._samples = deque()
        self.clear('not published')

    def clear(self, reason):
        with self._lock:
            self.active = False
            self.reason = str(reason)
            self._reference = None
            self._started = self._last_publish = -math.inf
            self._target = 0.
            self._rolling = False
            self._minimum_rtf = 1.
            self._samples.clear()

    def _observation(self, ego, now, reference):
        age = float(ego.get('age_s', math.inf))
        if 'received' in ego:
            receipt_age = now-float(ego['received'])
            if not 0. <= receipt_age <= .2:
                return None
            age = max(age, receipt_age)
        x, y = float(ego['x']), float(ego['y'])
        heading = math.radians(float(ego['heading_deg']))
        stamp, speed = float(ego['stamp']), float(ego['speed_kmh'])/3.6
        accel, brake = float(ego['accel']), float(ego['brake'])
        if (not all(math.isfinite(v) for v in (now, age, x, y, heading, stamp, speed, accel, brake))
                or not 0. <= age <= .2 or stamp <= 0. or not 0. <= speed <= 45.
                or not 0. <= accel <= 1. or not 0. <= brake <= 1.):
            return None
        station, distance, tangent, _ = _project([[x, y]], reference)
        axis = np.array([math.cos(heading), math.sin(heading)])
        if distance[0] > 1. or float(axis@tangent[0]) < math.cos(math.radians(25.)):
            return None
        return _Observation(stamp, np.array([x, y]), float(station[0]), speed, accel, brake, now)

    def _record(self, value):
        if self._samples:
            previous = self._samples[-1]
            dt = value.stamp-previous.stamp
            if dt < -1e-6 or value.now < previous.now:
                self.clear('source or observation clock reset')
                return False
            if dt <= 1e-6:
                same = (np.linalg.norm(value.xy-previous.xy) <= 1e-6
                        and max(abs(value.speed-previous.speed), abs(value.accel-previous.accel),
                                abs(value.brake-previous.brake)) <= 1e-6)
                if not same:
                    self.clear('conflicting repeated source pose')
                return same
            travelled = value.station-previous.station
            movement = float(np.linalg.norm(value.xy-previous.xy))
            if (dt > .35 or dt > value.now-previous.now+.2
                    or travelled < -.03
                    or movement > max(value.speed, previous.speed)*dt+.1
                    or (self._rolling and value.speed < previous.speed-.15)
                    or (self._rolling and travelled+.03 < .5*RTF_MIN*min(value.speed, previous.speed)*dt)):
                self.clear('measured launch progress inconsistent')
                return False
        self._samples.append(value)
        while value.stamp-self._samples[0].stamp > .35:
            self._samples.popleft()
        return True

    def published(self, world_path, ego, target_mps, now, proof_expires_at,
                  proof_valid=True, emergency=False):
        """Call only after success; a pending initial brake grants no prediction."""
        with self._lock:
            try:
                target, now, expires = float(target_mps), float(now), float(proof_expires_at)
                geometry = _geometry(world_path)
                if (not proof_valid or emergency or geometry is None
                        or not all(math.isfinite(v) for v in (target, now, expires)) or expires <= now):
                    self.clear('no successful positive fresh proof')
                    return False
                value = self._observation(ego, now, geometry)
                if value is None or target <= value.speed+.15:
                    self.clear('publication target or pose unsuitable')
                    return False
                if self.active and (not 0. <= now-self._last_publish <= .35
                                    or not _compatible(self._reference, geometry)):
                    self.clear('positive command gap or changed path')
                if not self.active:
                    self._reference = geometry
                    self._started = now
                    self.active = True
                value = self._observation(ego, now, self._reference)
                if value is None or not self._record(value):
                    self.clear('publication source inconsistent')
                    return False
                if self._rolling and (value.brake > .001 or value.accel <= 0.):
                    self.clear('measured braking or throttle release')
                    return False
                self._last_publish, self._target = now, target
                if value.brake <= .001 and value.accel > 0.:
                    self._rolling = self._rolling or value.speed > .15
                self.reason = 'successful positive fresh junction proof'
                return True
            except (KeyError, TypeError, ValueError, OverflowError, IndexError):
                self.clear('invalid publication input')
                return False

    def prediction(self, world_path, ego, now, rtf):
        """Return observed initial conditions; every safety decision must run again."""
        with self._lock:
            if not self.active:
                return None
            try:
                now, rtf = float(now), float(rtf)
                geometry = _geometry(world_path)
                if (not math.isfinite(now) or not math.isfinite(rtf) or not RTF_MIN <= rtf <= 1.
                        or not 0. <= now-self._last_publish <= .35 or geometry is None
                        or not _compatible(self._reference, geometry)):
                    self.clear('prediction command age or path invalid')
                    return None
                value = self._observation(ego, now, self._reference)
                if value is None or self._target <= value.speed or not self._record(value):
                    self.clear('prediction pose or progress invalid')
                    return None
                self._minimum_rtf = min(self._minimum_rtf, rtf)
                elapsed = max(0., now-self._started)*self._minimum_rtf
                if value.brake > .001 or value.accel <= 0.:
                    # The first positive command is sent while the prior WAIT
                    # brake may still be reported. Keep only a short pending
                    # seed; it cannot change a traffic prediction or authorize GO.
                    if self._rolling or elapsed > LAUNCH_SLOW[1]:
                        self.clear('braking or no throttle acknowledgement')
                    return None
                self._rolling = self._rolling or value.speed > .15
                moving = [sample for sample in self._samples if sample.brake <= .001 and sample.accel > 0.]
                initial = 0.
                if len(moving) >= 2 and moving[-1].stamp-moving[0].stamp >= .1-1e-6:
                    span = moving[-1].stamp-moving[0].stamp
                    progress = max(0., moving[-1].station-moving[0].station)
                    if progress >= .03:
                        # ds/dwall is below simulator speed for RTF<=1. Keep
                        # the lower observed value and an explicit .15m/s reserve.
                        initial = max(0., min(value.speed, *(sample.speed for sample in moving), progress/span)-.15)
                return dict(v_mps=initial, dead_s=max(0., LAUNCH_SLOW[1]-elapsed), stamp=value.stamp)
            except (KeyError, TypeError, ValueError, OverflowError, IndexError):
                self.clear('invalid prediction input')
                return None
