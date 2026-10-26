"""Bind control geometry, sensor time and safety decisions to one rolling plan."""
from dataclasses import dataclass, replace
import copy
import hashlib
import math
import time

import numpy as np


def static_reverse_release_ready(state, plan, forward_ready):
    """A static escape releases on three distinct, consecutive clear scans."""
    # Bind the policy to the escape, rather than allowing a later missing or
    # non-static plan to remove a static escape's forward proof requirement.
    static = state.get('static_recovery')
    if static is None:
        static = bool(plan is not None and plan.validation.get('static_rear_axle'))
        if static:
            state['static_recovery'] = True
    if not static:
        return bool(forward_ready)
    if (plan is None or not plan.validation.get('static_rear_axle') or not forward_ready
            or plan.validation.get('static_complete') is not True):
        state.pop('static_clear_scans', None)
        return False
    stamp = float(plan.sensor_stamp)
    scans = state.setdefault('static_clear_scans', [])
    if scans and (stamp < scans[-1] or stamp-scans[-1] > .35):
        scans.clear()
    if not scans or stamp > scans[-1]:
        scans.append(stamp)
        del scans[:-3]
    return len(scans) == 3


@dataclass(frozen=True)
class DrivingPlan:
    plan_id: int
    path: np.ndarray
    control_ego: dict
    validation_ego: dict
    sensor_stamp: float
    sensor_age_s: float
    checked_at: float
    allowed_mps: float
    forward_clear_m: float
    emergency: bool
    validation: dict
    mode: str = 'TUNNEL'
    stop_reason: str = None

    def __post_init__(self):
        path = np.array(self.path, dtype=np.float64, copy=True)[:, :2]
        path.setflags(write=False)
        object.__setattr__(self, 'path', path)
        for name in ('control_ego', 'validation_ego', 'validation'):
            object.__setattr__(self, name, copy.deepcopy(dict(getattr(self, name))))

    def path_digest(self):
        return hashlib.sha256(self.path.tobytes()).hexdigest()[:16]

    def frame_digest(self):
        values = [float(ego.get(key, 0.)) for ego in (self.control_ego, self.validation_ego)
                  for key in ('x', 'y', 'heading_deg', 'stamp')]
        values.append(float(self.sensor_stamp))
        return hashlib.sha256(np.asarray(values, dtype=np.float64).tobytes()).hexdigest()[:16]

    def checks_match_path(self):
        digest = self.path_digest()
        frame = self.frame_digest()
        return (all(self.validation.get(key, digest) == digest
                    for key in ('geometry_path_digest', 'traffic_path_digest')) and
                all(self.validation.get(key, frame) == frame
                    for key in ('geometry_frame_digest', 'traffic_frame_digest')))

    def fresh(self, now=None, max_age_s=.6):
        now = time.monotonic() if now is None else now
        elapsed = now-self.checked_at
        source_age = float(self.control_ego.get('stamp', 0.))-self.sensor_stamp
        return bool(self.sensor_stamp > 0. and math.isfinite(source_age) and
                    -.12 <= source_age <= max_age_s and 0. <= elapsed <= max_age_s and
                    math.isfinite(self.sensor_age_s) and
                    0. <= self.sensor_age_s+elapsed <= max_age_s and self.junction_motion_fresh(now))

    def junction_motion_fresh(self, now=None):
        """Positive entry commands cannot outlive their source-bound proof."""
        proofs = [self.validation[key] for key in ('junction_motion', 'junction_live', 'junction_recovery')
                  if key in self.validation]
        if not proofs or self.allowed_mps <= 0.:
            return True
        try:
            now = time.monotonic() if now is None else float(now)
            return all(math.isfinite(float(proof['expires_at'])) and
                       self.checked_at <= now <= float(proof['expires_at'])
                       for proof in proofs)
        except (KeyError, TypeError, ValueError, OverflowError):
            return False

    def matches(self, ego, path):
        path = np.asarray(path)
        return (path.ndim == 2 and path.shape[0] == len(self.path) and path.shape[1] >= 2
                and np.array_equal(path[:, :2], self.path)
                and all(abs(float(ego.get(key, 0.))-float(self.control_ego.get(key, 0.))) < 1e-6
                        for key in ('x', 'y', 'heading_deg', 'stamp')))

    def forward_ready(self, ego, now=None):
        required = max(8., abs(float(ego.get('speed_kmh', 0.)))/3.6*1.5+4.)
        return bool(self.fresh(now, .25) and self.checks_match_path() and self.validation.get('valid') and
                    (not self.validation.get('static_rear_axle') or
                     self.validation.get('static_complete') is True) and
                    not self.validation.get('blocked') and self.validation.get('pose_valid') and
                    self.validation.get('steering_valid') and not self.emergency and
                    math.isfinite(self.allowed_mps) and self.allowed_mps > 0. and
                    self.forward_clear_m >= required and len(self.path) >= 3 and
                    np.isfinite(self.path).all() and
                    np.linalg.norm(np.diff(self.path, axis=0), axis=1).sum() >= required)

    def decided(self, *, allowed_mps=None, emergency=None, forward_clear_m=None,
                mode=None, stop_reason=None, **validation):
        """Keep the geometry and source times when adding decisions for that path."""
        return replace(self, allowed_mps=self.allowed_mps if allowed_mps is None else allowed_mps,
                       emergency=self.emergency if emergency is None else emergency,
                       forward_clear_m=self.forward_clear_m if forward_clear_m is None else forward_clear_m,
                       mode=self.mode if mode is None else mode,
                       stop_reason=stop_reason, validation=dict(self.validation, **validation))

    def summary(self):
        return dict(plan_id=self.plan_id, sensor_stamp=self.sensor_stamp,
                    sensor_age_s=self.sensor_age_s, source_stamp=self.control_ego.get('stamp', 0.),
                    checked_at=self.checked_at, allowed_mps=self.allowed_mps,
                    forward_clear_m=self.forward_clear_m, emergency=self.emergency,
                    stop_reason=self.stop_reason, mode=self.mode, validation=dict(self.validation),
                    path_digest=self.path_digest())
