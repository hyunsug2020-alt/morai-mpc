"""Measured planar velocity with rejection of short-baseline tracking noise."""
import numpy as np


def measured_velocity(samples, max_speed_mps=8.0):
    if len(samples) < 4:
        return None
    recent = np.asarray([s for s in samples if samples[-1][0] - s[0] <= 0.8], dtype=float)
    if len(recent) < 4 or recent[-1, 0] - recent[0, 0] < 0.3 - 1e-9:
        return None
    dt = recent[None, :, 0] - recent[:, None, 0]
    pairs = dt >= 0.3 - 1e-9
    slopes = (recent[None, :, 1:3] - recent[:, None, 1:3])[pairs] / dt[pairs, None]
    velocity = np.median(slopes, axis=0)
    if not np.isfinite(velocity).all() or np.linalg.norm(velocity) > max_speed_mps:
        return None
    elapsed = recent[:, 0] - recent[-1, 0]
    intercepts = recent[:, 1:3] - elapsed[:, None] * velocity
    residual = float(np.median(np.linalg.norm(intercepts - np.median(intercepts, axis=0), axis=1)))
    displacement = float(np.linalg.norm(velocity) * (recent[-1, 0] - recent[0, 0]))
    if displacement < max(0.15, 3.0 * residual):
        return 0.0, 0.0
    return float(velocity[0]), float(velocity[1])
