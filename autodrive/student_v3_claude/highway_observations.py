"""Validate semantic highway observations before they can request a hard stop."""
from collections import deque
import math

import numpy as np

from motion_filter import measured_velocity
from sim_time import clip_rtf
from planning.highway.object_policy import EGO_HALF_LENGTH_M, MOTION_SOURCE_INDEX


def source_lag_s(stamp, now):
    """Missing stamps retain receipt-age compatibility; invalid clocks fail closed."""
    if stamp == 0.:
        return 0.
    if not math.isfinite(stamp) or not math.isfinite(now) or stamp < 0. or now-stamp < -.05:
        return 999.
    return max(0., now-stamp)


class HighwayObservationFilter:
    """Count distinct camera observations and reject unsupported tracking jumps.

    Raw LiDAR collision checks remain independent. Pedestrians, vehicles, and
    observations without a source timestamp retain their existing protection.
    """
    MAX_GAP_S = .35
    CONFIRM_S = .15
    CONFIRM_FRAMES = 3
    SHAPE_ANGLE_DEG = 20.
    SHAPE_SIZE_RATIO = 1.5

    def __init__(self):
        self.tracks = {}
        self.last_stamp = None

    @staticmethod
    def supported(obj, points, ego):
        if points is None or not len(points):
            return False
        h = math.radians(float(ego["heading_deg"]))
        delta = np.asarray(obj[:2])-np.array([ego["x"], ego["y"]])
        centre = delta @ np.array([[math.cos(h), -math.sin(h)],
                                   [math.sin(h), math.cos(h)]])
        yaw = math.radians(float(obj[8])-float(ego["heading_deg"]))
        rel = np.asarray(points)-centre
        along = rel @ [math.cos(yaw), math.sin(yaw)]
        across = rel @ [-math.sin(yaw), math.cos(yaw)]
        return int(np.sum((abs(along) <= float(obj[6])/2.+.2) &
                          (abs(across) <= float(obj[7])/2.+.2))) >= 2

    @staticmethod
    def stationary(obj, heading):
        row = list(obj)
        row[5], row[8] = 0., heading
        if len(row) > 10:
            row[9:11] = [0., 0.]
        return tuple(row)

    @staticmethod
    def propagate(obj, age_s):
        if float(obj[5]) <= .5 or len(obj) <= 10:
            return obj
        row = list(obj)
        row[:2] = (np.asarray(obj[:2])+np.asarray(obj[9:11])*
                   min(.5, max(0., age_s))).tolist()
        return tuple(row)

    @staticmethod
    def shape_difference(shape, reference):
        angle = abs((float(shape[2])-float(reference[2])+90.) % 180.-90.)
        dims = np.maximum(.3, np.asarray(shape[:2], dtype=float))
        old = np.maximum(.3, np.asarray(reference[:2], dtype=float))
        return angle, float(np.max(np.maximum(dims/old, old/dims)))

    def validate_shape(self, obj, state, stamp, continuous, info):
        shape = tuple(obj[6:9])
        # PCA may exchange its length/width axes or reverse an axis by 180
        # degrees. Compare equivalent rectangles before testing a shape jump.
        swapped = (shape[1], shape[0], shape[2]+90.)
        def error(value):
            angle, ratio = self.shape_difference(value, state["shape"])
            return angle/self.SHAPE_ANGLE_DEG+math.log(ratio)/math.log(self.SHAPE_SIZE_RATIO)
        if error(swapped) < error(shape):
            shape = swapped
        angle, ratio = self.shape_difference(shape, state["shape"])
        radians = math.radians(angle)
        projection = (shape[0]*abs(math.cos(radians))+shape[1]*abs(math.sin(radians)),
                      shape[0]*abs(math.sin(radians))+shape[1]*abs(math.cos(radians)))
        nested = all(size <= previous+1e-6 for size, previous in zip(projection, state['shape'][:2]))
        # A narrower current measurement must not inherit an inflated first
        # box. Raw LiDAR and near-field memory independently retain collision
        # protection for any measured points outside the smaller rectangle.
        if angle <= self.SHAPE_ANGLE_DEG and (ratio <= self.SHAPE_SIZE_RATIO or nested):
            state.update(shape=shape, shape_pending=None, shape_count=0)
        else:
            pending = state["shape_pending"]
            stable = (pending is not None and continuous and
                      self.shape_difference(shape, pending)[0] <= 15. and
                      self.shape_difference(shape, pending)[1] <= 1.25)
            if not stable:
                state.update(shape_pending=shape, shape_since=stamp, shape_count=0)
            state["shape_count"] += 1
            if (state["shape_count"] >= self.CONFIRM_FRAMES and
                    stamp-state["shape_since"] >= self.CONFIRM_S-1e-6):
                state.update(shape=shape, shape_pending=None, shape_count=0)
            else:
                info["shape_rejected"] += 1
        row = list(obj)
        row[6:9] = state["shape"]
        row[2] = max(row[6:8])
        return tuple(row)

    def update(self, objects, stamp, points, ego, age_s=0.):
        info = dict(pending=0, position_rejected=0, motion_rejected=0, shape_rejected=0,
                    source_age_s=round(float(age_s), 3), reused_frame=False)
        if stamp is None or not math.isfinite(stamp) or stamp <= 0.:
            return list(objects), [], info
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.tracks.clear()
            self.last_stamp = None
        previous_stamp = self.last_stamp
        new_frame = stamp != previous_stamp
        info["reused_frame"] = not new_frame
        if new_frame:
            self.tracks = {key: state for key, state in self.tracks.items()
                           if stamp-state["seen"] <= self.MAX_GAP_S}
        accepted, tentative = [], []
        for obj in objects:
            if obj[3] != "fusion_obstacle" or obj[4] != "obstacle" or len(obj) <= 11:
                accepted.append(obj)
                continue
            key = int(obj[11])
            state = self.tracks.get(key)
            supported = self.supported(obj, points, ego)
            if state is not None and new_frame:
                dt = stamp-state["seen"]
                velocity = np.asarray(state["clean"][9:11], dtype=float)
                predicted = np.asarray(state["clean"][:2])+velocity*dt
                if (np.linalg.norm(np.asarray(obj[:2])-predicted) > .6+max(4., float(obj[5]))*dt
                        and not supported):
                    info["position_rejected"] += 1
                    # A rejected measurement cannot renew a confirmed track.
                    if state["confirmed"]:
                        row = list(state["clean"])
                        row[:2] = predicted.tolist()
                        accepted.append(self.propagate(tuple(row), age_s))
                    else:
                        tentative.append(obj)
                    continue
            if state is None:
                state = dict(seen=stamp, since=stamp, count=0, history=[], confirmed=False,
                             heading=float(obj[8]), clean=obj, motion_count=0,
                             motion_since=stamp, motion_vector=None, shape=tuple(obj[6:9]),
                             shape_pending=None, shape_count=0, shape_since=stamp)
                self.tracks[key] = state
            if new_frame:
                continuous = state["seen"] == previous_stamp
                if not state["confirmed"] and not continuous:
                    state.update(since=stamp, count=0)
                state["count"] += 1
                state["seen"] = stamp
                state["history"] = [p for p in state["history"] if stamp-p[0] <= .8]
                state["history"].append((stamp, float(obj[0]), float(obj[1])))
                clean = obj
                if float(obj[5]) > .5:
                    measured = measured_velocity(state["history"], max_speed_mps=4.)
                    reported = np.asarray(obj[9:11], dtype=float)
                    previous_motion = state["motion_vector"]
                    if (not continuous or previous_motion is None or
                            np.linalg.norm(reported-previous_motion) > .8):
                        state.update(motion_count=0, motion_since=stamp)
                    state["motion_count"] += 1
                    state["motion_vector"] = reported
                    motion_confirmed = (state["motion_count"] >= self.CONFIRM_FRAMES and
                                        stamp-state["motion_since"] >= self.CONFIRM_S-1e-6)
                    if (not motion_confirmed or measured is None or np.linalg.norm(measured) <= .5 or
                            np.linalg.norm(np.asarray(measured)-reported) > .8):
                        clean = self.stationary(obj, state["heading"])
                        info["motion_rejected"] += 1
                else:
                    state.update(motion_count=0, motion_since=stamp, motion_vector=None)
                if float(ego.get("speed_kmh", 0.)) > 10.:
                    clean = self.validate_shape(clean, state, stamp, continuous, info)
                else:
                    state.update(shape=tuple(clean[6:9]), shape_pending=None, shape_count=0)
                state["heading"] = float(clean[8])
                state["clean"] = clean
                state["confirmed"] |= (state["count"] >= self.CONFIRM_FRAMES and
                                       stamp-state["since"] >= self.CONFIRM_S-1e-6)
            state["confirmed"] |= supported
            clean = state["clean"]
            if state['shape_pending'] is not None:
                # Presence confirmation does not validate a changing footprint.
                # Retaining its old rectangle at the new centroid can itself
                # invent a collision. Raw LiDAR remains the immediate veto;
                # pending geometry requests the same cautious slowdown as a
                # new detection until consecutive shape observations agree.
                tentative.append(clean)
                continue
            if state["confirmed"] or float(ego.get("speed_kmh", 0.)) <= 10.:
                # Positions are already in map coordinates. Only confirmed
                # movers need propagation from capture time to decision time.
                accepted.append(self.propagate(clean, age_s))
            else:
                tentative.append(clean)
        if new_frame:
            self.last_stamp = stamp
        info["pending"] = len(tentative)
        return accepted, tentative, info


# --- LiDAR cars without a detector velocity (highway only) -------------------
# User rule (2026-10-11): on the highway every vehicle is a moving car; a car
# without a measured velocity is never planned as a standing body.
# Run follow-20261010T184845Z, lap 1, 03:53:01.7-05.5: cars 37-55 m ahead were
# boxed at PointPillars score 0.23-0.66 by a 5-7 Hz detector. Lateral centroid
# jumps of up to 1.8 m restarted the detector's unvalidated history and the uid
# churned (500028/500029/500030/500031), so velocity_valid stayed false for ~4 s
# although 500029 was seen 8 times in 1.4 s at ~9.4 m/s (wall). As standing
# boxes they became a 48 m stop line at 82 km/h during a lane change ("highway
# braking trajectory conflict", 84 -> 44 km/h; lap 72.6 s against 52.9 s).
#
# Assumed speed (simulator km/h) without any measurement: the slowest highway
# NPC's desired speed in 404_contest.json (40 km/h; the others 50/60, no lane
# changes, so platoons settle at >= 40). It is also the upper bound of the
# assumption: a neighbour's measured speed never raises it (NPC speeds differ),
# while slower measured flow nearby (spawn acceleration, merge slowdown) lowers
# it down to UNMEASURED_CAR_MIN_KMH.
UNMEASURED_CAR_KMH = 40.
# Never standing. Slower bodies are owned by the raw LiDAR check: within 35 m
# every return counts, and farther returns that stay within 0.5 m for 0.12 s
# (FarReturnPersistence: < ~4 m/s measured = 18-25 km/h simulated at RTF
# 0.6-0.8) are confirmed static. Those returns are never masked by this car.
UNMEASURED_CAR_MIN_KMH = 20.
# Only cars wholly ahead get the assumption; beside/behind keep the existing
# unknown-speed vetoes (rear warm-up at centres < 5 m, target-lane rear gates).
UNMEASURED_AHEAD_CENTRE_M = 5.
VEHICLE_LANE_MATCH_M = 2.1          # same bound as HighwayLaneMap.motion_candidates
VEHICLE_SAME_FLOW_DEG = 45.
NEARBY_FLOW_ALONG_M = 40.
NEARBY_FLOW_LATERAL_M = 5.25        # own and adjacent lanes (1.5 x 3.5 m)
# Copy-side along-lane measurement over the uid's (or bridged) detections. Lane
# projection ignores the lateral centroid jitter that reset the detector; the
# window is stricter than the detector's 3 samples / 0.3 s.
VEHICLE_MEASURE_SAMPLES = 4
VEHICLE_MEASURE_SPAN_S = .5
VEHICLE_MEASURE_WINDOW_S = 1.2
VEHICLE_MEASURE_PAIR_S = .3
VEHICLE_MEASURE_RESIDUAL_M = 1.
VEHICLE_MEASURE_MAX_MPS = 45.
# uid churn: a new uid continues a dropped track found near its prediction.
VEHICLE_HISTORY_KEEP_S = 1.5
VEHICLE_BRIDGE_GAP_S = 1.2          # detector TRACK_KEEP_S (1 s) plus one scan
VEHICLE_BRIDGE_LATERAL_M = 1.5      # observed centroid jitter +-0.9 m; lanes 3.5 m apart
VEHICLE_BRIDGE_ALONG_M = 2.5
VEHICLE_BRIDGE_SPEED_FRACTION = .5  # along-lane uncertainty of the predicted travel


def lane_direction(lanes, xy):
    """(unit travel tangent, centreline distance) of the nearest HD-map lane point."""
    tree, points = getattr(lanes, 'tree', None), getattr(lanes, 'points', None)
    link_of = getattr(lanes, 'link_of', None)
    if tree is None or points is None or link_of is None or not len(points):
        return None
    try:
        distance, k = tree.query(np.asarray(xy, dtype=float)[:2])
    except (TypeError, ValueError):
        return None
    k = int(k)
    if not math.isfinite(float(distance)) or not 0 <= k < len(points):
        return None
    before = k-1 if k > 0 and link_of[k-1] == link_of[k] else k
    after = k+1 if k+1 < len(points) and link_of[k+1] == link_of[k] else k
    tangent = np.asarray(points[after], dtype=float)[:2]-np.asarray(points[before], dtype=float)[:2]
    norm = float(np.linalg.norm(tangent))
    if after == before or not math.isfinite(norm) or norm < 1e-6:
        return None
    return tangent/norm, float(distance)


def traffic_lane(lanes, xy, forward):
    """(tangent, distance) of a same-flow lane within VEHICLE_LANE_MATCH_M, else None."""
    lane = lane_direction(lanes, xy)
    if (lane is None or lane[1] > VEHICLE_LANE_MATCH_M or
            float(lane[0] @ forward) < math.cos(math.radians(VEHICLE_SAME_FLOW_DEG))):
        return None
    return lane


def ahead_of_ego(obj, origin, forward):
    """Ego-frame centre of a body wholly ahead of the front bumper (centre > 5 m), else None."""
    local = (np.asarray(obj[:2], dtype=float)-origin) @ np.column_stack(
        (forward, [-forward[1], forward[0]]))
    if local[0] <= UNMEASURED_AHEAD_CENTRE_M or local[0]-float(obj[6])/2. <= EGO_HALF_LENGTH_M:
        return None
    return local


def along_lane_speed(history, tangent):
    """Robust along-lane speed (measured units) of position samples, or None."""
    if len(history) < VEHICLE_MEASURE_SAMPLES:
        return None
    samples = np.asarray(history, dtype=float)
    samples = samples[samples[-1, 0]-samples[:, 0] <= VEHICLE_MEASURE_WINDOW_S+1e-9]
    if (len(samples) < VEHICLE_MEASURE_SAMPLES or
            samples[-1, 0]-samples[0, 0] < VEHICLE_MEASURE_SPAN_S-1e-9):
        return None
    along = (samples[:, 1:3]-samples[-1, 1:3]) @ np.asarray(tangent, dtype=float)
    dt = samples[None, :, 0]-samples[:, None, 0]
    pairs = dt >= VEHICLE_MEASURE_PAIR_S-1e-9
    if not pairs.any():
        return None
    speed = float(np.median((along[None, :]-along[:, None])[pairs]/dt[pairs]))
    if not math.isfinite(speed) or abs(speed) > VEHICLE_MEASURE_MAX_MPS:
        return None
    intercepts = along-(samples[:, 0]-samples[-1, 0])*speed
    if float(np.median(np.abs(intercepts-np.median(intercepts)))) > VEHICLE_MEASURE_RESIDUAL_M:
        return None
    return max(0., speed)


class HighwayVehicleMotion:
    """Forward motion for highway LiDAR cars ahead without a detector velocity.

    Every fresh detection extends a per-car history keyed by detector uid; a new
    uid near a dropped track's predicted position (along its lane) continues
    that history. A car ahead whose detector velocity is not valid receives,
    along its HD-map lane: the along-lane speed measured from that history
    ('tracked'), else the assumed highway flow ('assumed', see
    UNMEASURED_CAR_KMH). Detector-measured cars, cars beside/behind, off-lane
    and other-flow bodies are unchanged. Velocities are measured (wall) units,
    like the detector's: the assumption is converted with the runtime RTF.
    Marked cars (index 16) never mask LiDAR returns or own camera boxes.
    """
    def __init__(self):
        self.tracks = {}
        self.keys = {}
        self.stamp = None
        self.info = {}
        # Measured (detector or tracked) car motion of the latest update().
        self.flows = []

    def reset(self):
        self.tracks.clear()
        self.keys.clear()
        self.stamp = None

    @staticmethod
    def _fresh(car):
        try:
            return (car[3] == 'lidar_car' and len(car) > 15 and float(car[15]) == 0. and
                    np.isfinite(np.asarray(car[:2], dtype=float)).all())
        except (TypeError, ValueError):
            return False

    def _bridge(self, car, stamp, fresh_uids, claimed):
        position = np.asarray(car[:2], dtype=float)
        best = None
        for key, track in self.tracks.items():
            gap = stamp-track['seen']
            if (key in claimed or track['uid'] in fresh_uids or track.get('tangent') is None or
                    not 0. < gap <= VEHICLE_BRIDGE_GAP_S):
                continue
            tangent = track['tangent']
            normal = np.array([-tangent[1], tangent[0]])
            delta = position-track['position']-track['velocity']*gap
            along, lateral = abs(float(delta @ tangent)), abs(float(delta @ normal))
            along_gate = (VEHICLE_BRIDGE_ALONG_M+VEHICLE_BRIDGE_SPEED_FRACTION *
                          float(np.linalg.norm(track['velocity']))*gap)
            if lateral > VEHICLE_BRIDGE_LATERAL_M or along > along_gate:
                continue
            cost = along/along_gate+lateral/VEHICLE_BRIDGE_LATERAL_M
            if best is None or cost < best[0]:
                best = cost, key
        return None if best is None else best[1]

    def _observe(self, cars, stamp, info):
        """Extend per-car histories with this scan's fresh detections."""
        self.tracks = {key: track for key, track in self.tracks.items()
                       if stamp-track['seen'] <= VEHICLE_HISTORY_KEEP_S}
        self.keys = {uid: key for uid, key in self.keys.items() if key in self.tracks}
        fresh = [car for car in cars if self._fresh(car)]
        fresh_uids = {car[11] for car in fresh}
        claimed, new = set(), []
        for car in fresh:
            key = self.keys.get(car[11])
            if key in self.tracks and key not in claimed:
                claimed.add(key)
                self._append(self.tracks[key], car, stamp)
            else:
                new.append(car)
        for car in new:
            key = self._bridge(car, stamp, fresh_uids, claimed)
            if key is None:
                key = ('uid', car[11])
                self.tracks[key] = dict(history=deque(maxlen=32), uid=car[11], seen=stamp,
                                        position=np.asarray(car[:2], dtype=float),
                                        velocity=np.zeros(2), tangent=None)
            else:
                info['bridged'].append(dict(from_uid=self.tracks[key]['uid'], uid=car[11],
                    gap_s=round(stamp-self.tracks[key]['seen'], 3)))
                self.tracks[key]['uid'] = car[11]
            self.keys[car[11]] = key
            claimed.add(key)
            self._append(self.tracks[key], car, stamp)

    @staticmethod
    def _append(track, car, stamp):
        position = np.asarray(car[:2], dtype=float)
        if not track['history'] or stamp > track['history'][-1][0]:
            track['history'].append((stamp, float(position[0]), float(position[1])))
        track.update(seen=stamp, position=position)

    def update(self, cars, stamp, ego, lanes, rtf=1.):
        """Return cars (same order) and diagnostics; see the class docstring."""
        cars = list(cars)
        info = dict(inferred=[], bridged=[], assumed_kmh=UNMEASURED_CAR_KMH)
        self.info = info
        try:
            stamp = float(stamp)
        except (TypeError, ValueError):
            stamp = float('nan')
        valid_stamp = math.isfinite(stamp) and stamp > 0.
        if valid_stamp and self.stamp is not None and stamp < self.stamp:
            self.reset()
        if valid_stamp and stamp != self.stamp:
            self._observe(cars, stamp, info)
            self.stamp = stamp
        scale = clip_rtf(rtf)
        try:
            heading = math.radians(float(ego['heading_deg']))
            origin = np.array([float(ego['x']), float(ego['y'])])
        except (KeyError, TypeError, ValueError):
            return cars, info
        forward = np.array([math.cos(heading), math.sin(heading)])
        # First pass: lane, measured motion and fresh-track velocity of every car.
        rows, flows = [], []
        self.flows = flows
        for car in cars:
            if car[3] != 'lidar_car' or car[4] != 'npc' or len(car) < 16:
                rows.append(None)
                continue
            track = self.tracks.get(self.keys.get(car[11]))
            fresh = self._fresh(car)
            lane = traffic_lane(lanes, car[:2], forward)
            if track is not None and fresh and lane is not None:
                track['tangent'] = lane[0]
            if car[14]:
                velocity = np.asarray(car[9:11], dtype=float)
                if np.isfinite(velocity).all():
                    flows.append((car[11], np.asarray(car[:2], dtype=float), velocity))
                    if track is not None and fresh:
                        track['velocity'] = velocity
                rows.append(None)
                continue
            if lane is None:
                if track is not None and fresh:
                    track['velocity'] = np.zeros(2)
                rows.append(None)
                continue
            tracked = along_lane_speed(track['history'], lane[0]) if track is not None else None
            if tracked is not None:
                flows.append((car[11], np.asarray(car[:2], dtype=float), lane[0]*tracked))
            rows.append((track, fresh, lane[0], tracked))
        # Second pass: the assumption from measured flow nearby, then the output.
        result = []
        for car, row in zip(cars, rows):
            if row is None:
                result.append(car)
                continue
            track, fresh, tangent, speed = row
            source, nearby = 'tracked', 0
            if speed is None:
                source, speed, nearby = 'assumed', *self.assumed_speed(car, tangent, flows, scale)
            velocity = tangent*speed
            if track is not None and fresh:
                track['velocity'] = velocity
            local = ahead_of_ego(car, origin, forward)
            if local is None:
                result.append(car)
                continue
            age = float(car[15])
            out = list(car)+[None]*max(0, MOTION_SOURCE_INDEX+1-len(car))
            if 0. < age <= VEHICLE_BRIDGE_GAP_S:
                # The detector keeps an unvalidated track at its last position.
                out[0:2] = (np.asarray(car[:2], dtype=float)+velocity*age).tolist()
            out[5], out[9], out[10] = float(speed), float(velocity[0]), float(velocity[1])
            out[14], out[MOTION_SOURCE_INDEX] = True, source
            result.append(tuple(out))
            along = float((np.asarray(out[0:2], dtype=float)-origin) @ forward)
            info['inferred'].append(dict(uid=car[11], source=source,
                kmh=round(speed/scale*3.6, 1), along_m=round(along, 1),
                age_s=round(age, 3), nearby=nearby))
        return result, info

    def camera_vehicles(self, objects, ego, lanes, rtf=1., age_s=0.):
        """Unowned camera parked-vehicle boxes ahead in a same-flow lane move.

        The camera tracker flips a moving car between a measured 'fusion_npc'
        and a 'fusion_parked_vehicle' without motion (id 123 at 03:53:01-05 in
        the run above: 9.7-15 m/s, then 0). A vehicle box on the highway is
        the same unmeasured vehicle as above: the assumed flow along its lane,
        propagated by the camera age (<= 0.5 s), marked 'assumed'. Call after
        highway_objects, so LiDAR-owned duplicates are already removed.
        Obstacle-class boxes, off-lane, beside/behind and other-flow bodies
        are unchanged.
        """
        objects = list(objects)
        try:
            heading = math.radians(float(ego['heading_deg']))
            origin = np.array([float(ego['x']), float(ego['y'])])
            age = min(.5, max(0., float(age_s)))
        except (KeyError, TypeError, ValueError):
            return objects
        forward = np.array([math.cos(heading), math.sin(heading)])
        scale = clip_rtf(rtf)
        result = []
        for obj in objects:
            try:
                parked = (obj[3] == 'fusion_parked_vehicle' and len(obj) >= 11 and
                          not float(obj[5]) >= .5 and
                          np.isfinite(np.asarray(obj[:2], dtype=float)).all() and
                          min(float(obj[6]), float(obj[7])) > 0.)
            except (TypeError, ValueError, IndexError):
                parked = False
            lane = traffic_lane(lanes, obj[:2], forward) if parked else None
            if lane is None or ahead_of_ego(obj, origin, forward) is None:
                result.append(obj)
                continue
            speed, nearby = self.assumed_speed(obj, lane[0], self.flows, scale)
            velocity = lane[0]*speed
            out = list(obj)+[None]*max(0, MOTION_SOURCE_INDEX+1-len(obj))
            out[0:2] = (np.asarray(obj[:2], dtype=float)+velocity*age).tolist()
            out[4], out[5], out[9], out[10] = 'npc', float(speed), float(velocity[0]), float(velocity[1])
            out[14], out[15], out[MOTION_SOURCE_INDEX] = True, 0., 'assumed'
            result.append(tuple(out))
            self.info.setdefault('camera_assumed', []).append(dict(
                id=obj[11] if len(obj) > 11 else None, kmh=round(speed/scale*3.6, 1), nearby=nearby))
        return result

    @staticmethod
    def assumed_speed(car, tangent, flows, scale):
        """(measured-unit speed, nearby count) of the highway flow around a car."""
        normal = np.array([-tangent[1], tangent[0]])
        position = np.asarray(car[:2], dtype=float)
        speeds = []
        for uid, other, velocity in flows:
            if uid == car[11]:
                continue
            delta = other-position
            if (abs(float(delta @ tangent)) <= NEARBY_FLOW_ALONG_M and
                    abs(float(delta @ normal)) <= NEARBY_FLOW_LATERAL_M):
                forward = float(velocity @ tangent)
                if forward >= -.5:
                    speeds.append(max(0., forward))
        default = UNMEASURED_CAR_KMH/3.6*scale
        if not speeds:
            return default, 0
        return (min(default, max(UNMEASURED_CAR_MIN_KMH/3.6*scale, float(np.median(speeds)))),
                len(speeds))
