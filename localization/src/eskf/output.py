import numpy as np


class PositionCorrectionLimiter:
    """Keep published translation continuous while the estimator recovers.

    Velocity propagates motion in physical time. Only the residual position
    correction is limited, in metres per source-clock second. Estimator state
    is never modified; the unapplied residual remains visible to diagnostics.
    """

    def __init__(self, correction_rate=2.0):
        if not np.isfinite(correction_rate) or correction_rate <= 0.0:
            raise ValueError("correction_rate must be finite and positive")
        self.correction_rate = float(correction_rate)
        self.position = None
        self.velocity = None
        self.stamp = None
        self.remaining = np.zeros(2)
        self.last_step = np.zeros(2)

    def reset(self):
        """Permit an explicit, confirmed stationary scenario reset."""
        self.position = self.velocity = self.stamp = None
        self.remaining[:] = 0.0
        self.last_step[:] = 0.0

    def update(self, position, velocity, stamp, physical_dt):
        position = np.asarray(position, dtype=float)
        velocity = np.asarray(velocity, dtype=float)
        if (position.shape != (2,) or velocity.shape != (2,)
                or not np.all(np.isfinite(position))
                or not np.all(np.isfinite(velocity))
                or not np.isfinite(stamp) or not np.isfinite(physical_dt)):
            raise ValueError("output position, velocity and time must be finite")
        self.last_step[:] = 0.0
        if self.position is None:
            self.position = position.copy()
            self.velocity = velocity.copy()
            self.stamp = float(stamp)
        elif stamp > self.stamp:
            predicted = self.position + (
                0.5 * (self.velocity + velocity) * max(0.0, physical_dt))
            residual = position - predicted
            distance = float(np.linalg.norm(residual))
            maximum = self.correction_rate * (stamp - self.stamp)
            self.last_step = residual * min(1.0, maximum / max(distance, 1e-12))
            self.position = predicted + self.last_step
            self.velocity = velocity.copy()
            self.stamp = float(stamp)
        elif stamp == self.stamp:
            # Several callbacks can publish at the same physical instant.
            # None may spend correction budget a second time.
            self.velocity = velocity.copy()
        self.remaining = position - self.position
        return self.position.copy()
