"""Tunnel route scope and model-trajectory policy."""
import math

import numpy as np


class TunnelRoutePolicy:
    """Bound tunnel-only behavior to route stations and an exit-side rejoin zone."""

    def __init__(self, route_xy, start_xy, end_xy, rejoin_buffer_m=45.0, enabled=True):
        route = np.asarray(route_xy, dtype=float)
        ends = np.asarray((start_xy, end_xy), dtype=float)
        if route.ndim != 2 or route.shape[1] < 2 or len(route) < 2:
            raise ValueError("route_xy must be an Nx2 metric route")
        route = route[:, :2]
        if not np.isfinite(route).all() or ends.shape != (2, 2) or not np.isfinite(ends).all():
            raise ValueError("route and tunnel endpoints must be finite x,y coordinates")
        if not math.isfinite(rejoin_buffer_m) or rejoin_buffer_m < 0.0:
            raise ValueError("rejoin_buffer_m must be finite and non-negative")
        self.route_xy = route
        self.route_cumulative = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        indices = [int(np.argmin(np.sum((route - xy) ** 2, axis=1))) for xy in ends]
        self.lo, self.hi = sorted(indices)
        if self.lo == self.hi:
            raise ValueError("tunnel entry and exit must map to different route points")
        self.rejoin_buffer_m = float(rejoin_buffer_m)
        self.enabled = bool(enabled)

    def state(self, nearest_route_index):
        """Return tunnel scope, centerline permission, and exit-rejoin phase."""
        index = int(nearest_route_index)
        active = self.enabled and self.lo <= index <= self.hi
        exit_station = float(self.route_cumulative[self.hi])
        rejoin_station = max(float(self.route_cumulative[self.lo]), exit_station - self.rejoin_buffer_m)
        station = float(self.route_cumulative[index]) if 0 <= index < len(self.route_cumulative) else -1.0
        rejoin_phase = bool(active and station >= rejoin_station)
        return {
            "tunnel_active": active,
            "cross_center_allowed": active,
            "defer_rejoin": bool(active and not rejoin_phase),
            "rejoin_phase": rejoin_phase,
            "rejoin_distance_m": max(0.0, rejoin_station - station),
            "rejoin_start_index": int(np.searchsorted(self.route_cumulative, rejoin_station)),
            "from_index": self.lo,
            "to_index": self.hi,
        }


class TunnelModelPolicy:
    """Validate and use learned trajectory directly inside configured tunnel."""

    def __init__(self, route_xy, start_xy, end_xy, max_kmh=50.0, enabled=False):
        route = np.asarray(route_xy, dtype=float)[:, :2]
        ends = np.asarray((start_xy, end_xy), dtype=float)
        if ends.shape != (2, 2) or not np.isfinite(ends).all():
            raise ValueError("tunnel entry/exit must be finite x,y coordinates")
        if not math.isfinite(max_kmh) or not 5.0 <= max_kmh <= 50.0:
            raise ValueError("tunnel model speed limit must be 5..50 km/h")
        indices = [int(np.argmin(np.sum((route - xy) ** 2, axis=1))) for xy in ends]
        self.lo, self.hi = sorted(indices)
        if self.lo == self.hi:
            raise ValueError("tunnel entry and exit must be different route points")
        self.enabled, self.max_kmh = bool(enabled), float(max_kmh)

    def active(self, nearest_idx):
        return self.enabled and self.lo <= int(nearest_idx) <= self.hi

    def plan(self, predicted_path, dt, action, probability):
        path = np.asarray(predicted_path, dtype=np.float32)
        info = dict(mode="STUDENT TUNNEL", reason="learned trajectory without map repair",
                    model_only=True, control_owner="student_v63_tunnel", signal_phase=None,
                    action_intent=action, raw_peak_m=0.0, raw_end_m=0.0, applied_peak_m=0.0,
                    tunnel=dict(from_idx=self.lo, to_idx=self.hi, max_kmh=self.max_kmh),
                    model_speed_mps=0.0, target_speed_mps=0.0, stop_reason=None, motion_state="STOP")

        def stop(reason):
            info.update(reason=reason, stop_reason=reason)
            return path, info

        if (path.ndim != 2 or path.shape[1] < 2 or len(path) < 3
                or not np.isfinite(path).all() or not math.isfinite(dt) or dt <= 0.0):
            return stop("invalid learned trajectory")
        xy = path[:, :2]
        steps = np.diff(xy, axis=0)
        lengths = np.linalg.norm(steps, axis=1)
        if np.any(steps[:, 0] < -0.2) or np.any(lengths > 30.0 * dt + 0.5):
            return stop("reverse or discontinuous learned trajectory")
        model_speed = float(np.median(lengths[:max(1, min(len(lengths), round(1.0 / dt)))]) / dt)
        info["model_speed_mps"] = model_speed
        if action == "STOP" and probability >= 0.5:
            return stop("model STOP")
        if float(lengths.sum()) < 0.5 or model_speed < 0.1:
            return stop("model stationary trajectory")
        target = min(model_speed, self.max_kmh / 3.6)
        info.update(target_speed_mps=target, motion_state="DRIVE")
        return path.copy(), info


class TunnelLaneHold:
    """Remember model-selected lateral side until a checked exit rejoin succeeds."""

    def __init__(self, default_offset_m=2.2):
        if not math.isfinite(default_offset_m) or default_offset_m < 1.5:
            raise ValueError("default_offset_m must be finite and at least 1.5 m")
        self.default_offset_m = float(default_offset_m)
        self.value = None

    def observe_model(self, action, probability, tunnel_state):
        if (tunnel_state.get("defer_rejoin") and probability >= 0.5
                and action in ("LEFT_CHANGE", "RIGHT_CHANGE")):
            self.value = {
                "direction": 1.0 if action == "LEFT_CHANGE" else -1.0,
                "offset_m": self.default_offset_m,
            }
        return dict(self.value) if self.value else None

    def observe_plan(self, plan, tunnel_state):
        if not plan or plan.get("blocked"):
            return dict(self.value) if self.value else None
        if (plan.get("kind") == "rejoin"
                and (tunnel_state.get("rejoin_phase") or not tunnel_state.get("tunnel_active"))):
            self.value = None
        elif tunnel_state.get("tunnel_active") and plan.get("kind") != "rejoin":
            offset = abs(float(plan.get("offset_m", 0.0)))
            if offset >= 1.5:
                self.value = {
                    "direction": 1.0 if plan.get("direction", 0.0) >= 0.0 else -1.0,
                    "offset_m": offset,
                }
        return dict(self.value) if self.value else None
