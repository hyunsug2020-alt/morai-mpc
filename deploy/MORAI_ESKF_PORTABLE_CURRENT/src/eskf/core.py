import copy
import math

import numpy as np

from .util import wrap_angle


class RobustPlanarESKF:
    """Planar error-state filter with IMU bias estimation and robust updates."""

    PX = 0
    PY = 1
    VX = 2
    VY = 3
    YAW = 4
    BAX = 5
    BAY = 6
    BGZ = 7
    SIZE = 8

    def __init__(self, config=None, gps_from_imu=None, imu_lever=None):
        config = config or {}
        self.accel_noise_std = float(config.get("accel_noise_std", 0.35))
        self.gyro_noise_std = float(config.get("gyro_noise_std", 0.015))
        self.accel_bias_rw_std = float(
            config.get("accel_bias_random_walk_std", 0.02))
        self.gyro_bias_rw_std = float(
            config.get("gyro_bias_random_walk_std", 0.002))
        self.gps_position_variance = float(
            config.get("gps_position_variance", 1.0))
        self.gps_velocity_variance_floor = float(
            config.get("gps_velocity_variance_floor", 0.5))
        self.wheel_speed_variance = float(
            config.get("wheel_speed_variance", 0.04))
        self.imu_yaw_variance = float(
            config.get("imu_yaw_variance", 0.02))
        self.nhc_variance = float(config.get("nhc_variance", 0.09))
        self.zupt_velocity_variance = float(
            config.get("zupt_velocity_variance", 0.01))
        self.slam_position_variance = float(
            config.get("slam_position_variance", 0.25))
        self.slam_yaw_variance = float(
            config.get("slam_yaw_variance", 0.0025))
        self.gps_nis_soft = float(config.get("gps_nis_soft", 5.991))
        self.gps_nis_hard = float(config.get("gps_nis_hard", 13.816))
        self.velocity_nis_soft = float(
            config.get("velocity_nis_soft", 5.991))
        self.velocity_nis_hard = float(
            config.get("velocity_nis_hard", 13.816))
        self.wheel_speed_nis_soft = float(
            config.get("wheel_speed_nis_soft", 3.841))
        self.wheel_speed_nis_hard = float(
            config.get("wheel_speed_nis_hard", 10.828))
        self.yaw_nis_soft = float(config.get("yaw_nis_soft", 3.841))
        self.yaw_nis_hard = float(config.get("yaw_nis_hard", 10.828))
        self.constraint_nis_hard = float(
            config.get("constraint_nis_hard", 10.828))
        self.slam_nis_soft = float(config.get("slam_nis_soft", 7.815))
        self.slam_nis_hard = float(config.get("slam_nis_hard", 16.266))
        self.odometry_nis_soft = float(
            config.get("odometry_nis_soft", 5.991))
        self.odometry_nis_hard = float(
            config.get("odometry_nis_hard", 13.816))
        self.adaptive_alpha = float(config.get("adaptive_alpha", 0.04))
        self.adaptive_scale_min = float(
            config.get("adaptive_scale_min", 0.5))
        self.adaptive_scale_max = float(
            config.get("adaptive_scale_max", 16.0))
        self.max_accel_bias = float(config.get("max_accel_bias", 3.0))
        self.max_gyro_bias = float(config.get("max_gyro_bias", 0.3))

        self.gps_from_imu = np.asarray(
            gps_from_imu if gps_from_imu is not None else [0.0, 0.0],
            dtype=float)
        self.imu_lever = np.asarray(
            imu_lever if imu_lever is not None else [0.0, 0.0],
            dtype=float)
        self.x = np.zeros(self.SIZE, dtype=float)
        self.p = np.diag([
            4.0, 4.0, 9.0, 9.0, 0.5,
            0.25, 0.25, 0.0025,
        ])
        self.initialized = False
        self.last_corrected_yaw_rate = 0.0
        self.measurement_scales = {
            "gps_position": 1.0,
            "gps_velocity": 1.0,
            "wheel_speed": 1.0,
            "imu_yaw": 1.0,
            "slam_pose": 1.0,
            "odometry_pose": 1.0,
        }
        self.last_nis = {
            "gps_position": None,
            "gps_velocity": None,
            "wheel_speed": None,
            "imu_yaw": None,
            "nhc": None,
            "zupt": None,
            "slam_pose": None,
            "odometry_pose": None,
        }
        self.counters = {
            "gps_accepted": 0,
            "gps_rejected": 0,
            "gps_reacquired": 0,
            "velocity_reanchored": 0,
            "velocity_accepted": 0,
            "velocity_rejected": 0,
            "wheel_speed_accepted": 0,
            "wheel_speed_rejected": 0,
            "yaw_accepted": 0,
            "yaw_rejected": 0,
            "nhc_accepted": 0,
            "nhc_rejected": 0,
            "zupt_accepted": 0,
            "imu_spikes": 0,
            "invalid_measurements": 0,
            "stale_gps": 0,
            "out_of_order_imu": 0,
            "slam_accepted": 0,
            "slam_rejected": 0,
            "slam_degenerate": 0,
            "slam_degenerate_accepted": 0,
            "slam_stationary_hints": 0,
            "slam_stale": 0,
            "slam_frame_jumps": 0,
            "odometry_accepted": 0,
            "odometry_rejected": 0,
        }

    def snapshot(self):
        """Return a deep filter snapshot suitable for deterministic replay."""
        return {
            "x": self.x.copy(),
            "p": self.p.copy(),
            "initialized": bool(self.initialized),
            "last_corrected_yaw_rate": float(
                self.last_corrected_yaw_rate),
            "measurement_scales": copy.deepcopy(self.measurement_scales),
            "last_nis": copy.deepcopy(self.last_nis),
            "counters": copy.deepcopy(self.counters),
        }

    def restore(self, snapshot):
        """Restore a snapshot produced by :meth:`snapshot`."""
        self.x = np.asarray(snapshot["x"], dtype=float).copy()
        self.p = np.asarray(snapshot["p"], dtype=float).copy()
        self.initialized = bool(snapshot["initialized"])
        self.last_corrected_yaw_rate = float(
            snapshot["last_corrected_yaw_rate"])
        self.measurement_scales = copy.deepcopy(
            snapshot["measurement_scales"])
        self.last_nis = copy.deepcopy(snapshot["last_nis"])
        self.counters = copy.deepcopy(snapshot["counters"])
        self._stabilize_covariance()

    def is_numerically_valid(self):
        """Check finite state and positive-semidefinite covariance."""
        if not np.all(np.isfinite(self.x)) or not np.all(np.isfinite(self.p)):
            return False
        if not np.allclose(self.p, self.p.T, atol=1e-8):
            return False
        return bool(np.min(np.linalg.eigvalsh(self.p)) >= -1e-9)

    @staticmethod
    def _rotation(yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return np.array([
            [cos_yaw, -sin_yaw],
            [sin_yaw, cos_yaw],
        ])

    def initialize(self, gps_position, yaw):
        yaw = wrap_angle(float(yaw))
        gps_position = np.asarray(gps_position, dtype=float)
        self.x.fill(0.0)
        self.x[self.YAW] = yaw
        self.x[0:2] = (
            gps_position - self._rotation(yaw) @ self.gps_from_imu)
        self.p = np.diag([
            self.gps_position_variance,
            self.gps_position_variance,
            4.0,
            4.0,
            max(self.imu_yaw_variance, 1e-4),
            0.25,
            0.25,
            0.0025,
        ])
        self.initialized = True

    def predict(self, acceleration_body, yaw_rate_measurement, dt):
        acceleration_body = np.asarray(acceleration_body, dtype=float)
        yaw = float(self.x[self.YAW])
        accel_unbiased = acceleration_body - self.x[self.BAX:self.BAY + 1]
        yaw_rate = float(yaw_rate_measurement) - self.x[self.BGZ]
        rotation = self._rotation(yaw)
        acceleration_map = rotation @ accel_unbiased
        d_accel_d_yaw = np.array([
            -math.sin(yaw) * accel_unbiased[0]
            - math.cos(yaw) * accel_unbiased[1],
            math.cos(yaw) * accel_unbiased[0]
            - math.sin(yaw) * accel_unbiased[1],
        ])

        self.x[0:2] += self.x[2:4] * dt + 0.5 * acceleration_map * dt**2
        self.x[2:4] += acceleration_map * dt
        self.x[self.YAW] = wrap_angle(yaw + yaw_rate * dt)
        self.last_corrected_yaw_rate = yaw_rate

        f = np.eye(self.SIZE)
        f[0:2, 2:4] = np.eye(2) * dt
        f[0:2, self.YAW] = 0.5 * d_accel_d_yaw * dt**2
        f[0:2, self.BAX:self.BAY + 1] = -0.5 * rotation * dt**2
        f[2:4, self.YAW] = d_accel_d_yaw * dt
        f[2:4, self.BAX:self.BAY + 1] = -rotation * dt
        f[self.YAW, self.BGZ] = -dt

        accel_var = self.accel_noise_std**2
        gyro_var = self.gyro_noise_std**2
        q = np.zeros((self.SIZE, self.SIZE), dtype=float)
        q[0:2, 0:2] = np.eye(2) * 0.25 * accel_var * dt**4
        q[0:2, 2:4] = np.eye(2) * 0.5 * accel_var * dt**3
        q[2:4, 0:2] = q[0:2, 2:4]
        q[2:4, 2:4] = np.eye(2) * accel_var * dt**2
        q[self.YAW, self.YAW] = gyro_var * dt**2
        q[self.BAX:self.BAY + 1, self.BAX:self.BAY + 1] = (
            np.eye(2) * self.accel_bias_rw_std**2 * dt)
        q[self.BGZ, self.BGZ] = self.gyro_bias_rw_std**2 * dt
        self.p = f @ self.p @ f.T + q
        self._stabilize_covariance()

    def _stabilize_covariance(self):
        self.p = 0.5 * (self.p + self.p.T)
        diagonal = np.maximum(np.diag(self.p), 1e-12)
        np.fill_diagonal(self.p, diagonal)

    def _inject(self, delta):
        self.x += delta
        self.x[self.YAW] = wrap_angle(self.x[self.YAW])
        self.x[self.BAX:self.BAY + 1] = np.clip(
            self.x[self.BAX:self.BAY + 1],
            -self.max_accel_bias, self.max_accel_bias)
        self.x[self.BGZ] = float(np.clip(
            self.x[self.BGZ], -self.max_gyro_bias, self.max_gyro_bias))

    def _update(self, innovation, h, r, name, soft_gate=None,
                hard_gate=None, angle=False, adaptive=False):
        innovation = np.atleast_1d(np.asarray(innovation, dtype=float))
        if angle:
            innovation[0] = wrap_angle(innovation[0])
        h = np.atleast_2d(np.asarray(h, dtype=float))
        r = np.atleast_2d(np.asarray(r, dtype=float))
        scale = self.measurement_scales.get(name, 1.0) if adaptive else 1.0
        r_scaled = r * scale
        s = h @ self.p @ h.T + r_scaled
        try:
            solved = np.linalg.solve(s, innovation)
        except np.linalg.LinAlgError:
            self.counters["invalid_measurements"] += 1
            return False, math.inf
        nis = float(innovation.T @ solved)
        self.last_nis[name] = nis

        if hard_gate is not None and nis > hard_gate:
            rejected_key = {
                "gps_position": "gps_rejected",
                "gps_velocity": "velocity_rejected",
                "wheel_speed": "wheel_speed_rejected",
                "imu_yaw": "yaw_rejected",
                "nhc": "nhc_rejected",
                "slam_pose": "slam_rejected",
                "odometry_pose": "odometry_rejected",
            }.get(name)
            if rejected_key in self.counters:
                self.counters[rejected_key] += 1
            return False, nis

        if soft_gate is not None and nis > soft_gate:
            r_scaled *= max(1.0, nis / soft_gate)
            s = h @ self.p @ h.T + r_scaled
        try:
            kalman_gain = np.linalg.solve(
                s.T, (self.p @ h.T).T).T
        except np.linalg.LinAlgError:
            self.counters["invalid_measurements"] += 1
            return False, nis

        delta = kalman_gain @ innovation
        self._inject(delta)
        identity = np.eye(self.SIZE)
        ikh = identity - kalman_gain @ h
        self.p = (
            ikh @ self.p @ ikh.T
            + kalman_gain @ r_scaled @ kalman_gain.T)
        self._stabilize_covariance()

        accepted_key = {
            "gps_position": "gps_accepted",
            "gps_velocity": "velocity_accepted",
            "wheel_speed": "wheel_speed_accepted",
            "imu_yaw": "yaw_accepted",
            "nhc": "nhc_accepted",
            "zupt": "zupt_accepted",
            "slam_pose": "slam_accepted",
            "odometry_pose": "odometry_accepted",
        }.get(name)
        if accepted_key in self.counters:
            self.counters[accepted_key] += 1

        if adaptive and name in self.measurement_scales:
            normalized_nis = nis / max(innovation.size, 1)
            target = float(np.clip(
                normalized_nis,
                self.adaptive_scale_min,
                self.adaptive_scale_max))
            old_scale = self.measurement_scales[name]
            self.measurement_scales[name] = float(np.clip(
                (1.0 - self.adaptive_alpha) * old_scale
                + self.adaptive_alpha * target,
                self.adaptive_scale_min,
                self.adaptive_scale_max))
        return True, nis

    def update_gps_position(self, gps_position, variance=None):
        gps_position = np.asarray(gps_position, dtype=float)
        yaw = float(self.x[self.YAW])
        lever_world = self._rotation(yaw) @ self.gps_from_imu
        expected = self.x[0:2] + lever_world
        h = np.zeros((2, self.SIZE), dtype=float)
        h[0:2, 0:2] = np.eye(2)
        h[:, self.YAW] = np.array([-lever_world[1], lever_world[0]])
        position_variance = (
            self.gps_position_variance if variance is None else variance)
        return self._update(
            gps_position - expected,
            h,
            np.eye(2) * position_variance,
            "gps_position",
            self.gps_nis_soft,
            self.gps_nis_hard,
            adaptive=True)

    def reanchor_gps_position(
            self, gps_position, variance=None, base_velocity=None,
            velocity_variance=None):
        """Reacquire a consensus GPS fix after a confirmed receiver outage."""
        gps_position = np.asarray(gps_position, dtype=float)
        yaw = float(self.x[self.YAW])
        self.x[0:2] = (
            gps_position - self._rotation(yaw) @ self.gps_from_imu)
        self.p[0:2, :] = 0.0
        self.p[:, 0:2] = 0.0
        position_variance = (
            self.gps_position_variance if variance is None else variance)
        self.p[self.PX, self.PX] = max(float(position_variance), 1e-4)
        self.p[self.PY, self.PY] = max(float(position_variance), 1e-4)
        self.p[self.VX, self.VX] = max(self.p[self.VX, self.VX], 4.0)
        self.p[self.VY, self.VY] = max(self.p[self.VY, self.VY], 4.0)
        self.measurement_scales["gps_position"] = 1.0
        self.measurement_scales["gps_velocity"] = 1.0
        self.counters["gps_reacquired"] += 1
        if base_velocity is not None:
            self.reanchor_base_velocity(
                base_velocity, variance=velocity_variance)
        self._stabilize_covariance()

    def reanchor_base_velocity(self, base_velocity, variance=None):
        """Reset map velocity from a separately validated base-link speed.

        The nominal velocity state belongs to the IMU point.  Convert the
        requested base-link velocity through the current lever arm so that
        :meth:`base_state` returns exactly the supplied velocity even while
        the vehicle is rotating.
        """
        base_velocity = np.asarray(base_velocity, dtype=float)
        if base_velocity.shape != (2,) or not np.all(np.isfinite(
                base_velocity)):
            raise ValueError("base_velocity must be a finite 2-vector")
        yaw = float(self.x[self.YAW])
        imu_world = self._rotation(yaw) @ self.imu_lever
        omega = self.last_corrected_yaw_rate
        rotational_correction = np.array([
            omega * imu_world[1],
            -omega * imu_world[0],
        ])
        self.x[self.VX:self.VY + 1] = (
            base_velocity - rotational_correction)
        self.p[self.VX:self.VY + 1, :] = 0.0
        self.p[:, self.VX:self.VY + 1] = 0.0
        velocity_variance = 1.0 if variance is None else float(variance)
        velocity_variance = max(velocity_variance, 1.0e-4)
        self.p[self.VX, self.VX] = velocity_variance
        self.p[self.VY, self.VY] = velocity_variance
        self.measurement_scales["gps_velocity"] = 1.0
        self.measurement_scales["wheel_speed"] = 1.0
        self.counters["velocity_reanchored"] += 1
        self._stabilize_covariance()

    def update_gps_velocity(self, velocity_map, variance):
        h = np.zeros((2, self.SIZE), dtype=float)
        h[0:2, 2:4] = np.eye(2)
        return self._update(
            np.asarray(velocity_map, dtype=float) - self.x[2:4],
            h,
            np.eye(2) * max(variance, self.gps_velocity_variance_floor),
            "gps_velocity",
            self.velocity_nis_soft,
            self.velocity_nis_hard,
            adaptive=True)

    def update_wheel_speed(self, forward_speed, variance=None):
        """Constrain base-link longitudinal speed without pose leakage."""
        yaw = float(self.x[self.YAW])
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx = float(self.x[self.VX])
        vy = float(self.x[self.VY])
        expected = (
            cos_yaw * vx + sin_yaw * vy
            + self.last_corrected_yaw_rate * self.imu_lever[1])
        h = np.zeros((1, self.SIZE), dtype=float)
        h[0, self.VX] = cos_yaw
        h[0, self.VY] = sin_yaw
        h[0, self.YAW] = -sin_yaw * vx + cos_yaw * vy
        h[0, self.BGZ] = -self.imu_lever[1]
        measurement_variance = (
            self.wheel_speed_variance if variance is None else variance)
        return self._update(
            [float(forward_speed) - expected],
            h,
            [[max(float(measurement_variance), 1.0e-4)]],
            "wheel_speed",
            self.wheel_speed_nis_soft,
            self.wheel_speed_nis_hard,
            adaptive=True)

    def update_yaw(self, yaw_measurement, variance=None):
        h = np.zeros((1, self.SIZE), dtype=float)
        h[0, self.YAW] = 1.0
        yaw_variance = self.imu_yaw_variance if variance is None else variance
        return self._update(
            [wrap_angle(yaw_measurement - self.x[self.YAW])],
            h,
            [[yaw_variance]],
            "imu_yaw",
            self.yaw_nis_soft,
            self.yaw_nis_hard,
            angle=True,
            adaptive=True)

    def update_nonholonomic_constraint(self, yaw_rate_measurement):
        yaw = float(self.x[self.YAW])
        vx = float(self.x[self.VX])
        vy = float(self.x[self.VY])
        measured_rate = float(yaw_rate_measurement)
        body_lateral_velocity = (
            -math.sin(yaw) * vx + math.cos(yaw) * vy
            - (measured_rate - self.x[self.BGZ]) * self.imu_lever[0])
        h = np.zeros((1, self.SIZE), dtype=float)
        h[0, self.VX] = -math.sin(yaw)
        h[0, self.VY] = math.cos(yaw)
        h[0, self.YAW] = -math.cos(yaw) * vx - math.sin(yaw) * vy
        h[0, self.BGZ] = self.imu_lever[0]
        return self._update(
            [-body_lateral_velocity],
            h,
            [[self.nhc_variance]],
            "nhc",
            hard_gate=self.constraint_nis_hard)

    def update_zero_velocity(self):
        h = np.zeros((2, self.SIZE), dtype=float)
        h[0:2, 2:4] = np.eye(2)
        return self._update(
            -self.x[2:4],
            h,
            np.eye(2) * self.zupt_velocity_variance,
            "zupt",
            hard_gate=self.gps_nis_hard)

    def update_slam_pose(self, base_position, yaw, position_variance,
                         yaw_variance):
        base_position = np.asarray(base_position, dtype=float)
        state_yaw = float(self.x[self.YAW])
        imu_world = self._rotation(state_yaw) @ self.imu_lever
        expected_position = self.x[0:2] - imu_world
        innovation = np.array([
            base_position[0] - expected_position[0],
            base_position[1] - expected_position[1],
            wrap_angle(yaw - state_yaw),
        ])
        h = np.zeros((3, self.SIZE), dtype=float)
        h[0:2, 0:2] = np.eye(2)
        h[0:2, self.YAW] = np.array([
            imu_world[1], -imu_world[0]])
        h[2, self.YAW] = 1.0
        r = np.diag([
            max(position_variance, self.slam_position_variance),
            max(position_variance, self.slam_position_variance),
            max(yaw_variance, self.slam_yaw_variance),
        ])
        return self._update(
            innovation,
            h,
            r,
            "slam_pose",
            self.slam_nis_soft,
            self.slam_nis_hard,
            adaptive=True)

    def update_slam_position(self, base_position, position_variance):
        base_position = np.asarray(base_position, dtype=float)
        state_yaw = float(self.x[self.YAW])
        imu_world = self._rotation(state_yaw) @ self.imu_lever
        expected_position = self.x[0:2] - imu_world
        h = np.zeros((2, self.SIZE), dtype=float)
        h[0:2, 0:2] = np.eye(2)
        h[0:2, self.YAW] = np.array([
            imu_world[1], -imu_world[0]])
        variance = max(position_variance, self.slam_position_variance)
        return self._update(
            base_position - expected_position,
            h,
            np.eye(2) * variance,
            "slam_pose",
            self.slam_nis_soft,
            self.slam_nis_hard,
            adaptive=True)

    def update_odometry_position(self, base_position, position_variance):
        """Weak base-link position update from aligned relative odometry."""
        base_position = np.asarray(base_position, dtype=float)
        state_yaw = float(self.x[self.YAW])
        imu_world = self._rotation(state_yaw) @ self.imu_lever
        expected_position = self.x[0:2] - imu_world
        h = np.zeros((2, self.SIZE), dtype=float)
        h[0:2, 0:2] = np.eye(2)
        h[0:2, self.YAW] = np.array([
            imu_world[1], -imu_world[0]])
        variance = max(float(position_variance), 1.0e-4)
        return self._update(
            base_position - expected_position,
            h,
            np.eye(2) * variance,
            "odometry_pose",
            self.odometry_nis_soft,
            self.odometry_nis_hard,
            adaptive=True)

    def base_state(self):
        yaw = float(self.x[self.YAW])
        rotation = self._rotation(yaw)
        imu_world = rotation @ self.imu_lever
        base_position = self.x[0:2] - imu_world
        omega = self.last_corrected_yaw_rate
        base_velocity_map = self.x[2:4] + np.array([
            omega * imu_world[1],
            -omega * imu_world[0],
        ])
        return base_position, base_velocity_map
