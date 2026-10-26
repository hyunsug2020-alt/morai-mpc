"""Measured pedestrian motion history and moving-object point exclusion."""
import math

import numpy as np


class PedestrianMotionHistory:
    """Recent speed peaks and uninterrupted standing time per perceived person."""

    def __init__(self, keep_s=2.):
        self.keep_s = keep_s
        self.tracks = {}
        self.now = None

    @staticmethod
    def key(obj):
        return ("id", obj[11]) if len(obj) > 11 and obj[11] is not None else ("name", obj[3])

    def update(self, objects, now):
        if self.now is not None and now < self.now:
            self.tracks.clear()
        self.now = now
        self.tracks = {k: v for k, v in self.tracks.items() if now - v["last"] <= self.keep_s}
        for obj in objects:
            if obj[4] != "pedestrian":
                continue
            key = self.key(obj)
            track = self.tracks.get(key)
            if track is None or math.hypot(obj[0] - track["xy"][0], obj[1] - track["xy"][1]) > 3.:
                track = dict(samples=[], standing=None)
            speed = abs(float(obj[5]))
            track["samples"] = [(t, v) for t, v in track["samples"] if now - t <= self.keep_s]
            track["samples"].append((now, speed))
            # Standing is judged on world displacement, not the reported speed: a static
            # cyclist model's speed flickered 0<->0.5+ m/s 210 times in 400 s and kept the
            # car in stop-and-go the whole time (auto run 440). Moved < 0.4 m over the
            # last 2 s = standing; a genuine walk covers > 1 m in that window.
            track.setdefault("xys", [])
            track["xys"] = [(t, x, y) for t, x, y in track["xys"] if now - t <= self.keep_s]
            track["xys"].append((now, float(obj[0]), float(obj[1])))
            span = now - track["xys"][0][0]
            moved = max(math.hypot(float(obj[0]) - x, float(obj[1]) - y) for _, x, y in track["xys"])
            walking = (moved >= .4 if span >= 1.0 else speed >= .5)
            track["standing"] = (None if walking else
                                 now if track["standing"] is None else track["standing"])
            track.update(last=now, xy=obj[:2])
            self.tracks[key] = track

    def max_speed(self, obj):
        track = self.tracks.get(self.key(obj))
        if track and track.get("xys") and track["xys"][-1][0] - track["xys"][0][0] >= 1.0:
            # A confirmed stander (see update) reports its true speed, not the jitter.
            span = track["xys"][-1][0] - track["xys"][0][0]
            moved = max(math.hypot(track["xys"][-1][1] - x, track["xys"][-1][2] - y) for _, x, y in track["xys"])
            if moved < .4:
                return min(abs(float(obj[5])), moved / span)
        return max([abs(float(obj[5]))] + [v for _, v in track["samples"]]) if track else abs(float(obj[5]))

    def standing_for(self, obj, now):
        track = self.tracks.get(self.key(obj))
        return now - track["standing"] if track and track["standing"] is not None else 0.


def without_moving_objects(points, objects, ego, pedestrian_motion=None):
    """Exclude tracked moving-object footprints from static LiDAR bypass triggers."""
    if points is None or not len(points):
        return points
    keep = np.ones(len(points), dtype=bool)
    yaw = math.radians(float(ego["heading_deg"]))
    c, s = math.cos(yaw), math.sin(yaw)
    for obj in objects:
        speed = (pedestrian_motion.max_speed(obj) if obj[4] == "pedestrian" and pedestrian_motion is not None
                 else float(obj[5]))
        if obj[4] not in ("npc", "pedestrian") or speed <= (0.5 if obj[4] == "npc" else 0.3):
            continue
        dx, dy = float(obj[0]) - ego["x"], float(obj[1]) - ego["y"]
        ox, oy = c * dx + s * dy, -s * dx + c * dy
        h = math.radians(float(obj[8]) - float(ego["heading_deg"]))
        ch, sh = math.cos(h), math.sin(h)
        rel = points - (ox, oy)
        longitudinal = ch * rel[:, 0] + sh * rel[:, 1]
        lateral = -sh * rel[:, 0] + ch * rel[:, 1]
        extra_long, extra_lat = (1., 1.) if obj[4] == "pedestrian" else (.4, .35)
        keep &= (np.abs(longitudinal) > float(obj[6]) / 2 + extra_long) | (np.abs(lateral) > float(obj[7]) / 2 + extra_lat)
    return points[keep]
