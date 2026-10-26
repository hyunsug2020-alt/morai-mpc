import math
from collections import deque

import numpy as np

from .core import RobustPlanarESKF
from .util import wrap_angle


class SE2Alignment:
    """Robustly align a local SLAM frame with the global MORAI map frame."""

    def __init__(self, alpha=0.05, reset_distance=5.0,
                 reset_yaw=math.radians(20.0), reset_confirm_samples=3,
                 window_size=600, min_samples=10, min_baseline=12.0,
                 continuity_max_failures=5,
                 continuity_confirm_samples=10):
        self.alpha = float(alpha)
        self.reset_distance = float(reset_distance)
        self.reset_yaw = float(reset_yaw)
        self.reset_confirm_samples = int(reset_confirm_samples)
        self.min_samples = int(min_samples)
        self.min_baseline = float(min_baseline)
        self.continuity_max_failures = int(continuity_max_failures)
        self.continuity_confirm_samples = max(
            int(continuity_confirm_samples), self.reset_confirm_samples)
        self.continuity_target_count = self.continuity_confirm_samples
        self.translation = np.zeros(2, dtype=float)
        self.yaw = 0.0
        self.initialized = False
        self.ready = False
        self.recovering = False
        self.anchor_local_position = None
        self.anchor_stamp = None
        self.reset_count = 0
        self.correspondences = deque(maxlen=int(window_size))
        self.pending_position_correction = None
        self.pending_yaw = None
        self.pending_count = 0
        self.continuity_translation = None
        self.continuity_yaw = None
        self.continuity_count = 0
        self.continuity_failures = 0
        self.continuity_correspondences = []

    @staticmethod
    def _rotation(yaw):
        return RobustPlanarESKF._rotation(yaw)

    @staticmethod
    def _circular_mean(angles):
        angles = np.asarray(angles, dtype=float)
        return math.atan2(
            float(np.mean(np.sin(angles))),
            float(np.mean(np.cos(angles))))

    @staticmethod
    def _fit_rigid_transform(local_points, global_points):
        local_mean = np.mean(local_points, axis=0)
        global_mean = np.mean(global_points, axis=0)
        local_centered = local_points - local_mean
        global_centered = global_points - global_mean
        covariance = local_centered.T @ global_centered
        u, _, vt = np.linalg.svd(covariance)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0.0:
            vt[-1, :] *= -1.0
            rotation = vt.T @ u.T
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        translation = global_mean - rotation @ local_mean
        return translation, wrap_angle(yaw)

    def _fit_correspondences(self):
        if len(self.correspondences) < self.min_samples:
            return None
        local_points = np.asarray([
            sample[0] for sample in self.correspondences], dtype=float)
        global_points = np.asarray([
            sample[1] for sample in self.correspondences], dtype=float)
        orientation_differences = np.asarray([
            sample[2] for sample in self.correspondences], dtype=float)
        local_centered = local_points - np.mean(local_points, axis=0)
        baseline = float(np.max(np.linalg.norm(local_centered, axis=1)))

        if baseline >= self.min_baseline:
            translation, yaw = self._fit_rigid_transform(
                local_points, global_points)
            predicted = (
                (self._rotation(yaw) @ local_points.T).T
                + translation)
            residuals = np.linalg.norm(predicted - global_points, axis=1)
            median = float(np.median(residuals))
            mad = float(np.median(np.abs(residuals - median)))
            threshold = max(1.0, median + 3.0 * 1.4826 * mad)
            inliers = residuals <= threshold
            if np.count_nonzero(inliers) >= self.min_samples:
                translation, yaw = self._fit_rigid_transform(
                    local_points[inliers], global_points[inliers])
        else:
            yaw = self._circular_mean(orientation_differences)
            rotation = self._rotation(yaw)
            translations = (
                global_points - (rotation @ local_points.T).T)
            translation = np.median(translations, axis=0)
        return np.asarray(translation, dtype=float), wrap_angle(yaw)

    def _apply_fit(self):
        fitted = self._fit_correspondences()
        if fitted is None:
            return
        fitted_translation, fitted_yaw = fitted
        if self.ready:
            self.translation = (
                (1.0 - self.alpha) * self.translation
                + self.alpha * fitted_translation)
            self.yaw = wrap_angle(
                self.yaw
                + self.alpha * wrap_angle(fitted_yaw - self.yaw))
        else:
            self.translation = fitted_translation
            self.yaw = fitted_yaw
        self.ready = True
        self.recovering = False

    @property
    def continuity_pending(self):
        return self.continuity_translation is not None

    @property
    def orientation_yaw_estimate(self):
        if not self.correspondences:
            return None
        return self._circular_mean([
            sample[2] for sample in self.correspondences
        ])

    def cancel_continuity_recovery(self):
        self.continuity_translation = None
        self.continuity_yaw = None
        self.continuity_count = 0
        self.continuity_failures = 0
        self.continuity_correspondences = []
        self.continuity_target_count = self.continuity_confirm_samples

    def recover_continuity(self, global_position, global_yaw,
                           local_position, local_yaw, stamp, start=False,
                           confirm_samples=None):
        """Transfer a trusted transform across a persistent local-frame jump."""
        global_position = np.asarray(global_position, dtype=float)
        local_position = np.asarray(local_position, dtype=float)
        if start or not self.continuity_pending:
            candidate_yaw = wrap_angle(global_yaw - local_yaw)
            self.continuity_yaw = candidate_yaw
            self.continuity_translation = (
                global_position
                - self._rotation(candidate_yaw) @ local_position)
            self.continuity_count = 1
            self.continuity_failures = 0
            self.continuity_target_count = max(
                self.reset_confirm_samples,
                int(
                    self.continuity_confirm_samples
                    if confirm_samples is None else confirm_samples))
            self.continuity_correspondences = [(
                local_position.copy(),
                global_position.copy(),
                candidate_yaw)]
            return False

        predicted_position = (
            self.continuity_translation
            + self._rotation(self.continuity_yaw) @ local_position)
        predicted_yaw = wrap_angle(self.continuity_yaw + local_yaw)
        consistent = (
            np.linalg.norm(predicted_position - global_position)
            <= self.reset_distance * 3.0
            and abs(wrap_angle(predicted_yaw - global_yaw))
            <= self.reset_yaw)
        if not consistent:
            self.continuity_failures += 1
            if self.continuity_failures > self.continuity_max_failures:
                self.cancel_continuity_recovery()
            return False

        self.continuity_correspondences.append((
            local_position.copy(),
            global_position.copy(),
            wrap_angle(global_yaw - local_yaw)))
        self.continuity_yaw = self._circular_mean([
            sample[2] for sample in self.continuity_correspondences
        ])
        rotation = self._rotation(self.continuity_yaw)
        translations = np.asarray([
            sample_global - rotation @ sample_local
            for sample_local, sample_global, _
            in self.continuity_correspondences
        ])
        self.continuity_translation = np.median(
            translations, axis=0)
        self.continuity_count += 1
        if self.continuity_count < self.continuity_target_count:
            return False
        self.translation = self.continuity_translation
        self.yaw = self.continuity_yaw
        self.ready = True
        self.recovering = False
        self.reset_count += 1
        self.correspondences.clear()
        self.correspondences.append((
            local_position.copy(),
            global_position.copy(),
            wrap_angle(global_yaw - local_yaw)))
        self.anchor_local_position = local_position.copy()
        self.anchor_stamp = float(stamp)
        self.cancel_continuity_recovery()
        return True

    def update(self, global_position, global_yaw,
               local_position, local_yaw, stamp):
        global_position = np.asarray(global_position, dtype=float)
        local_position = np.asarray(local_position, dtype=float)
        candidate_yaw = wrap_angle(global_yaw - local_yaw)
        candidate_translation = (
            global_position
            - self._rotation(candidate_yaw) @ local_position)
        if not self.initialized:
            self.translation = candidate_translation
            self.yaw = candidate_yaw
            self.initialized = True
            self.correspondences.append((
                local_position.copy(),
                global_position.copy(),
                candidate_yaw))
        else:
            predicted_position, _ = self.transform(
                local_position, local_yaw)
            position_correction = global_position - predicted_position
            position_error = np.linalg.norm(position_correction)
            yaw_error = abs(wrap_angle(candidate_yaw - self.yaw))
            is_large_change = (
                len(self.correspondences) >= 2 * self.min_samples
                and (
                    position_error > self.reset_distance
                    or yaw_error > self.reset_yaw))
            if is_large_change:
                pending_is_consistent = (
                    self.pending_position_correction is not None
                    and np.linalg.norm(
                        position_correction
                        - self.pending_position_correction)
                    <= self.reset_distance * 0.5
                    and abs(wrap_angle(
                        candidate_yaw - self.pending_yaw))
                    <= self.reset_yaw * 0.5)
                if pending_is_consistent:
                    self.pending_count += 1
                    self.pending_position_correction = (
                        0.5 * self.pending_position_correction
                        + 0.5 * position_correction)
                    self.pending_yaw = wrap_angle(
                        self.pending_yaw
                        + 0.5 * wrap_angle(
                            candidate_yaw - self.pending_yaw))
                else:
                    self.pending_position_correction = position_correction
                    self.pending_yaw = candidate_yaw
                    self.pending_count = 1
                if self.pending_count < self.reset_confirm_samples:
                    return False
                reset_yaw = self.pending_yaw
                self.translation = (
                    global_position
                    - self._rotation(reset_yaw) @ local_position)
                self.yaw = reset_yaw
                self.reset_count += 1
                self.ready = False
                self.recovering = True
                self.correspondences.clear()
                self.pending_position_correction = None
                self.pending_yaw = None
                self.pending_count = 0
            else:
                self.pending_position_correction = None
                self.pending_yaw = None
                self.pending_count = 0
            self.correspondences.append((
                local_position.copy(),
                global_position.copy(),
                candidate_yaw))
            self._apply_fit()

        self.anchor_local_position = local_position.copy()
        self.anchor_stamp = float(stamp)
        return True

    def transform(self, local_position, local_yaw):
        local_position = np.asarray(local_position, dtype=float)
        global_position = (
            self.translation
            + self._rotation(self.yaw) @ local_position)
        global_yaw = wrap_angle(self.yaw + local_yaw)
        return global_position, global_yaw

    def distance_from_anchor(self, local_position):
        if self.anchor_local_position is None:
            return math.inf
        return float(np.linalg.norm(
            np.asarray(local_position, dtype=float)
            - self.anchor_local_position))


