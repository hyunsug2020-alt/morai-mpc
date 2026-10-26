from enum import Enum


class GpsMode(str, Enum):
    STARTING = "starting"
    NORMAL = "normal"
    GPS_LOST = "gps_lost"
    REACQUIRING = "reacquiring"
    DEGRADED = "degraded"


class GpsModeMachine:
    """Explicit receiver-health state independent of ROS callbacks."""

    def __init__(self, outage_timeout=1.5, rejection_limit=5,
                 recovery_accepts=3):
        self.outage_timeout = float(outage_timeout)
        self.rejection_limit = max(1, int(rejection_limit))
        self.recovery_accepts = max(1, int(recovery_accepts))
        self.mode = GpsMode.STARTING
        self.last_message_stamp = None
        self.last_accept_stamp = None
        self.consecutive_rejections = 0
        self.recovery_count = 0
        self.transitions = 0

    def _set_mode(self, mode):
        mode = GpsMode(mode)
        if mode != self.mode:
            self.mode = mode
            self.transitions += 1

    def initialized(self, stamp):
        stamp = float(stamp)
        self.last_message_stamp = stamp
        self.last_accept_stamp = stamp
        self.consecutive_rejections = 0
        self.recovery_count = 0
        self._set_mode(GpsMode.NORMAL)

    def tick(self, stamp):
        if self.mode == GpsMode.STARTING:
            return self.mode
        reference = (
            self.last_message_stamp
            if self.mode in (GpsMode.REACQUIRING, GpsMode.DEGRADED)
            else self.last_accept_stamp)
        if reference is None or float(stamp) - reference > self.outage_timeout:
            self.recovery_count = 0
            self._set_mode(GpsMode.GPS_LOST)
        return self.mode

    def received(self, stamp):
        stamp = float(stamp)
        self.tick(stamp)
        self.last_message_stamp = (
            stamp if self.last_message_stamp is None
            else max(stamp, self.last_message_stamp))
        if self.mode in (GpsMode.GPS_LOST, GpsMode.DEGRADED):
            self.recovery_count = 0
            self._set_mode(GpsMode.REACQUIRING)
        return self.mode

    def accepted(self, stamp):
        stamp = float(stamp)
        self.last_accept_stamp = (
            stamp if self.last_accept_stamp is None
            else max(stamp, self.last_accept_stamp))
        self.consecutive_rejections = 0
        if self.mode == GpsMode.REACQUIRING:
            self.recovery_count += 1
            if self.recovery_count >= self.recovery_accepts:
                self._set_mode(GpsMode.NORMAL)
        else:
            self.recovery_count = 0
            self._set_mode(GpsMode.NORMAL)
        return self.mode

    def reanchored(self, stamp):
        self.recovery_count = self.recovery_accepts
        stamp = float(stamp)
        self.last_accept_stamp = (
            stamp if self.last_accept_stamp is None
            else max(stamp, self.last_accept_stamp))
        self.consecutive_rejections = 0
        self._set_mode(GpsMode.NORMAL)
        return self.mode

    def rejected(self, stamp=None):
        if stamp is not None:
            stamp = float(stamp)
            self.last_message_stamp = (
                stamp if self.last_message_stamp is None
                else max(stamp, self.last_message_stamp))
        self.consecutive_rejections += 1
        if (self.mode == GpsMode.NORMAL
                and self.consecutive_rejections >= self.rejection_limit):
            self._set_mode(GpsMode.DEGRADED)
        return self.mode

    def diagnostics(self, now):
        self.tick(float(now))
        return {
            "gps_mode": self.mode.value,
            "gps_mode_transitions": self.transitions,
            "gps_consecutive_rejections": self.consecutive_rejections,
            "gps_recovery_accepts": self.recovery_count,
        }

