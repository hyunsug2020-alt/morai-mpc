"""Collision checks for the geometry actually sent to the follower.

This is a speed constraint, not another route planner. Source timestamps own
motion evidence; an unconfirmed velocity never proves that an object is parked.
"""
from dataclasses import dataclass
import math

import numpy as np

from ped_guard import (PathFrame, LAUNCH_DEAD_S, EGO_FRONT_M, BESIDE_MARGIN_M, BESIDE_TOWARD_MPS,
                       BESIDE_CLEARANCE_M)
from ego_body import EGO_HALF_LENGTH_M, ROAD_LEGACY_SHIFT_M
from lead_follow import EMERGENCY_DECEL_MPS2
from motion_filter import measured_velocity
from motion_limits import LAUNCH_ACCEL_MPS2
from sim_time import clip_rtf


@dataclass(frozen=True)
class SafetyResult:
    cap_mps: float
    emergency: bool
    clear_m: float
    reason: str
    validation: dict


def stop_cap(distance, lag_s, brake_mps2=2.5):
    distance = max(0., float(distance))
    lag_s = max(0., float(lag_s))
    return max(0., math.sqrt((brake_mps2 * lag_s) ** 2 +
                            2. * brake_mps2 * distance) - brake_mps2 * lag_s)


def travel_envelope(times, speed, target, lag_s, brake_mps2):
    """Distance interval reached by the executed cap, including launch response."""
    lag_s = max(lag_s, LAUNCH_DEAD_S) if speed < 1. else lag_s
    speed, target = max(0., speed), max(0., target)
    steps = np.diff(times)
    active_time = np.r_[0., np.cumsum(np.where(times[:-1] >= lag_s, steps, 0.))]
    delta = target-speed
    rates = np.array([LAUNCH_ACCEL_MPS2*.8, LAUNCH_ACCEL_MPS2*1.2]
                     if delta >= 0. else [brake_mps2, brake_mps2])
    velocity = speed+np.sign(delta)*np.minimum(abs(delta), rates[:, None]*active_time)
    distance = np.column_stack((np.zeros(2),
        np.cumsum(.5*(velocity[:, :-1]+velocity[:, 1:])*steps, axis=1)))
    return distance.min(axis=0), distance.max(axis=0)


# The planner bypasses a slower obstacle as static (model_avoidance.is_static)
# and gui.moving_conflict ignores any slower non-pedestrian. Fusion reaches
# 0.5-0.95 m/s on boxes and parked cars whose measured centres stay within
# their face jitter while the ego passes (follow-20261009T191227Z 97.7 s,
# 20261009T183356Z 118.6 s, 20261009T204445Z 394.7 s); a lower threshold here
# stopped beside objects the published bypass had already been checked around.
FUSED_STATIC_MPS = 1.
# Fusion publishes zero until its own 0.3 s displacement window is filled, so
# only a longer run of zero reports is measured rest.
FUSED_REST_S = .3
# People: a report at least this fast (measured m/s) is a walk the history fit
# cannot overrule; anything slower is judged by the robust history fit.
PERSON_WALK_REPORT_MPS = .8
# The newest position this far off the robust history line is a real step or
# start: keep the short-baseline provisional fit for it.
PERSON_STEP_M = .25
# A person measured slower than this (m/s) is not forecast along that velocity:
# 0.2-0.5 m/s is kerb-side position jitter, and over the 4 s horizon it walked
# a standing person 1-2 m into our corridor beside the body ("live path
# collision" every 1-2 s, follow-20261010T162527Z 01:27:50-28:00). Their possible
# start keeps the departure-readiness speed constraint below.
PERSON_FORECAST_MIN_MPS = .5
DEPARTURE_ACCEL_MPS2, DEPARTURE_MPS = 4., 1.8
# Approaching a person who could still start: reserve the follower's speed
# response in the stop envelope too. Without it the car tracked the shrinking cap
# ~1-3 km/h late and every kerb-side pass ended in a "collision" stop pulse
# (follow-20261010T162527Z 01:32:33.0 / 34.4 / 35.2 at 19 / 12 / 8 km/h).
DEPARTURE_APPROACH_LAG_S = .4


def departure_time(room, toward_mps, accel=DEPARTURE_ACCEL_MPS2, walk_mps=DEPARTURE_MPS):
    """Earliest time (s) a person covers ``room`` toward our corridor.

    ``toward_mps`` is their present velocity component toward the path; a person
    walking away has to stop before turning back.
    """
    room, t0 = max(0., float(room)), 0.
    u = float(toward_mps)
    if u < 0.:
        t0, room, u = -u/accel, room+u*u/(2.*accel), 0.
    top = max(walk_mps, u)
    ramp_t = (top-u)/accel
    ramp_d = u*ramp_t+.5*accel*ramp_t*ramp_t
    if room <= ramp_d:
        return t0+(math.sqrt(u*u+2.*accel*room)-u)/accel
    return t0+ramp_t+(room-ramp_d)/top


def fused_motion(velocity, prior, stamp):
    """(velocity, rest_since, onset) for one semantic-fusion non-person report.

    A box or parked car measured at rest whose visible face changes as the
    ego passes produced a single moving report, then left the fusion output;
    its memory forecast that report for 0.6 s (follow-20261009T204445Z
    101.5 s, 20261009T183356Z 96.2 s). A start from measured rest must be
    reported again within FUSED_REST_S before it is forecast, also when a
    report in between dips below the threshold. Motion first seen without
    measured rest and established motion are used immediately. MotionMemory
    forecasts a missing fused body only after distinct motion reports agree.
    """
    rest = prior.get('rest_since') if prior is not None else None
    onset = prior.get('onset') if prior is not None else None
    if onset is not None and stamp-onset > FUSED_REST_S:
        onset = None
    if float(np.linalg.norm(velocity)) < FUSED_STATIC_MPS:
        return np.zeros(2), stamp if rest is None else rest, onset
    if onset is None and rest is not None and prior['stamp']-rest >= FUSED_REST_S:
        return np.zeros(2), rest, stamp
    return velocity, None, None


class MotionMemory:
    """Keep unknown, standing and moving observations, including short dropouts."""
    def __init__(self):
        self.tracks = {}
        self.stamps = {}

    def update(self, source, objects, stamp, age_s, now):
        stamp, age_s = float(stamp), float(age_s)
        if not math.isfinite(stamp) or stamp <= 0. or not 0. <= age_s <= .6:
            return
        previous = self.stamps.get(source)
        if previous is not None and stamp < previous:
            self.tracks = {k: v for k, v in self.tracks.items() if k[0] != source}
        if previous == stamp:
            return  # rereading a scan cannot confirm motion or refresh its age
        self.stamps[source] = stamp
        current_keys = {(source, obj[11] if len(obj)>11 else obj[3]) for obj in objects if len(obj)>=11}
        for obj in objects:
            if len(obj) < 11 or not np.isfinite(np.asarray(obj[:2], float)).all():
                continue
            key = (source, obj[11] if len(obj) > 11 else obj[3])
            old = self.tracks.get(key)
            aliases = []
            # Fusion assigns IDs per class. A parked/moving label change must
            # not leave the previous ID predicting a second, ghost car beside
            # the fresh observation of the same measured body.
            if str(obj[3]).startswith('fusion_') and obj[4] != 'pedestrian':
                for other_key, other in self.tracks.items():
                    other_obj = other['object']
                    if (other_key in current_keys or other_key[0] != source or
                            (other_obj[4] == obj[4] and other_obj[3] == obj[3]) or
                            other_obj[4] == 'pedestrian' or not str(other_obj[3]).startswith('fusion_')):
                        continue
                    dimensions = np.asarray(obj[6:8], float)
                    same_size = np.all(np.abs(dimensions-other['size']) <=
                                       np.maximum(.2, .2*np.maximum(dimensions, other['size'])))
                    if same_size and np.linalg.norm(np.asarray(obj[:2])-other_obj[:2]) < .5:
                        aliases.append((other_key, other))
                for other_key, other in aliases:
                    if old is None or other['stamp'] > old['stamp']:
                        old = other
                    del self.tracks[other_key]
            # A slow apparent fragment wholly inside a fresh parked-car box
            # cannot be a separate body moving through that car. Keep its
            # present geometry under the static sweep, but do not forecast the
            # changing visible face across an already checked adjacent bypass.
            if (obj[3] == 'fusion_obstacle' and obj[4] == 'obstacle' and
                    max(obj[6:8]) <= 1.8 and min(obj[6:8]) <= 1.2 and
                    np.linalg.norm(obj[9:11]) < 1.):
                yaw = math.radians(float(obj[8]))
                axes = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
                corners = np.array([[-1., -1.], [-1., 1.], [1., -1.], [1., 1.]]) * np.asarray(obj[6:8])/2.
                corners = corners @ axes.T + obj[:2]
                for body in objects:
                    if body[3] != 'fusion_parked_vehicle' or max(body[6:8]) < 2.5:
                        continue
                    h = math.radians(float(body[8]))
                    body_axes = np.array([[math.cos(h), -math.sin(h)], [math.sin(h), math.cos(h)]])
                    local = (corners-body[:2]) @ body_axes
                    if np.all(np.abs(local) <= np.asarray(body[6:8])/2.+.1):
                        row = list(obj)
                        row[5], row[9], row[10] = 0., 0., 0.
                        obj = tuple(row)
                        break
            history = [] if old is None else old['history']
            metadata = obj[12] if len(obj)>12 and isinstance(obj[12], dict) else {}
            xy = np.asarray(metadata.get('motion_center', obj[:2]), float)
            size = np.asarray(obj[6:8], float)
            shape_changed = bool(old is not None and np.any(np.abs(size-old['size']) >
                                 np.maximum(.4, .25*np.maximum(size, old['size']))))
            # Fusion encodes its velocity direction in heading. At low speed
            # that direction can jump while the measured car body stays put.
            # Keep physical body rotation continuous; retain the supplied world
            # velocity separately so a real departure is still used immediately.
            vehicle = (obj[4] == 'npc' or 'vehicle' in str(obj[3]))
            if (old is not None and vehicle and obj[4] == 'npc' and str(obj[3]).startswith('fusion_') and
                    not shape_changed and 0. < stamp-old['stamp'] <= .6):
                angle = (math.radians(float(obj[8])-float(old['object'][8]))+math.pi/2.) % math.pi-math.pi/2.
                limit = 1.2*(stamp-old['stamp'])
                if abs(angle) > limit:
                    row = list(obj)
                    row[8] = float(old['object'][8])+math.degrees(float(np.clip(angle, -limit, limit)))
                    obj = tuple(row)
            if history and (stamp-history[-1][0] > .6 or
                            np.linalg.norm(xy-history[-1][1]) > 4. or shape_changed):
                history = []
            prior = old if history else None  # same continuous body evidence
            history = [(t, p) for t, p in history if stamp-t <= .8] + [(stamp, xy)]
            velocity = np.asarray(obj[9:11], float)
            if not np.isfinite(velocity).all():
                velocity = np.zeros(2)
            span = stamp-history[0][0]
            fit = (xy-history[0][1])/span if span >= .08 else np.zeros(2)
            if np.linalg.norm(fit) > 20.:
                fit = np.zeros(2)
            if obj[4] == 'pedestrian' and np.linalg.norm(fit) > 4.:
                fit *= 4./np.linalg.norm(fit)
            confirmed = len(history) >= 4 and span >= .3
            vehicle_valid = len(obj) > 14 and bool(obj[14])
            # Before confirmation, a measured displacement is provisional
            # motion, rather than a zero-speed promise. Keep the faster estimate.
            # TunnelTracks, verified LiDAR vehicle velocities and warmed-up
            # semantic fusion already reject changing cluster faces. Their
            # published, padded box centre is not independent motion evidence.
            # Re-fitting that centre revives rejected motion on concrete and
            # parked vehicles. People and unfiltered observations can still
            # supply provisional displacement before motion confirmation.
            fused = str(obj[3]).startswith('fusion_') and obj[4] != 'pedestrian'
            filtered_motion = (source in ('tunnel_lidar', 'tunnel_semantic') or
                               (source == 'vehicles' and vehicle_valid) or fused)
            rest_since = onset = None
            motion_report_stamp = None
            motion_confirmed = False
            reported = float(np.linalg.norm(velocity))
            if filtered_motion:
                fit = np.zeros(2)
                velocity_stamp = stamp
                if fused:
                    velocity, rest_since, onset = fused_motion(velocity, prior, stamp)
                    last_report = prior.get('motion_report_stamp') if prior is not None else None
                    if last_report is not None and 0. < stamp-last_report <= FUSED_REST_S:
                        motion_report_stamp = last_report
                        motion_confirmed = prior.get('motion_confirmed', False)
                    if reported >= FUSED_STATIC_MPS:
                        motion_confirmed = motion_report_stamp is not None
                        motion_report_stamp = stamp
                if (aliases and old is not None and np.linalg.norm(velocity) < .3 and
                        np.linalg.norm(old['velocity']) >= 1. and
                        stamp-old.get('velocity_stamp', old['stamp']) <= .3):
                    velocity = old['velocity'].copy()
                    velocity_stamp = old.get('velocity_stamp', old['stamp'])
                    rest_since = onset = None
            else:
                velocity_stamp = stamp
            if obj[4] == 'pedestrian' and not filtered_motion:
                # The endpoint fit over 0.08-0.8 s turned 0.1-0.2 m kerb-side
                # jitter into 0.2-0.6 m/s "walks". With a full history, use the
                # same robust fit as the receiver and fusion instead; it also
                # overrules a sub-walking report (< PERSON_WALK_REPORT_MPS) that
                # the positions do not support. Real walks, a new track and a
                # fresh step off the fitted line keep the original estimates.
                samples = [(t, p[0], p[1]) for t, p in history]
                robust = measured_velocity(samples, 4.)
                if robust is not None:
                    robust = np.asarray(robust, float)
                    track_t = np.asarray([t for t, _, _ in samples])-samples[-1][0]
                    track_xy = np.asarray([(x, y) for _, x, y in samples])
                    line = np.median(track_xy-track_t[:, None]*robust, axis=0)
                    if float(np.linalg.norm(track_xy[-1]-line)) <= PERSON_STEP_M:
                        fit = robust
                        if (reported < PERSON_WALK_REPORT_MPS and
                                np.linalg.norm(robust) < .5*reported):
                            velocity = robust.copy()
            if shape_changed and obj[4] == 'obstacle':
                velocity = np.zeros(2)
            if np.linalg.norm(velocity) < .3 and np.linalg.norm(fit) > np.linalg.norm(velocity):
                velocity = fit
            if fused and obj[4] == 'npc' and prior is not None and not np.any(velocity):
                # The fused heading is the direction of the rejected velocity.
                # A standing car cannot turn: keep its measured body heading.
                row = list(obj)
                row[8] = float(prior['object'][8])
                obj = tuple(row)
            speed = float(np.linalg.norm(velocity))
            state = ('moving' if speed >= .3 and (confirmed or vehicle_valid) else
                     'standing' if confirmed and speed < .2 and
                     (span >= .7 or (old is not None and old['state'] == 'standing')) else 'unknown')
            identity_ids = list(old.get('identity_ids', ())) if old is not None else []
            if key[1] not in identity_ids:
                identity_ids.append(key[1])
            self.tracks[key] = dict(object=tuple(obj), history=history, stamp=stamp,
                observed_at=now-age_s, velocity=velocity, state=state, size=size,
                confirmed=confirmed or vehicle_valid, velocity_stamp=velocity_stamp,
                identity_ids=tuple(identity_ids[-8:]), rest_since=rest_since, onset=onset, source=source,
                motion_report_stamp=motion_report_stamp, motion_confirmed=motion_confirmed,
                fusion_mps=reported if fused and reported >= .3 and speed < .3 else None)

        # A fresh source frame explicitly missing a once-reported fused mover
        # cannot turn that report into 0.6 s of continued travel. Retain its
        # last measured body and age; raw returns still own static collisions.
        # People and repeated non-person motion retain their dropout forecast.
        for key, track in self.tracks.items():
            obj = track['object']
            if (key[0] != source or key in current_keys or obj[4] == 'pedestrian' or
                    not str(obj[3]).startswith('fusion_') or track['motion_confirmed']):
                continue
            track['velocity'] = np.zeros(2)
            values = list(obj)
            values[5], values[9], values[10] = 0., 0., 0.
            track['object'] = tuple(values)
            if track['state'] == 'moving':
                track['state'] = 'unknown'

    def observations(self, now):
        self.tracks = {k: v for k, v in self.tracks.items()
                       if 0. <= now-v['observed_at'] <= .6}
        return list(self.tracks.values())


class PathSafety:
    def __init__(self, brake_mps2=2.5, response_s=.15, body_shift_m=ROAD_LEGACY_SHIFT_M):
        self.brake_mps2, self.response_s = brake_mps2, response_s
        # Body extent ahead of / behind the pose along the path (ego_body). The tunnel
        # passes REAR_AXLE_SHIFT_M (front 3.845 m, rear 0.79 m: the pose is the rear
        # axle); the road keeps its legacy +-2.3175 m (known 1.53 m error, tuned).
        self.front_m = EGO_HALF_LENGTH_M+float(body_shift_m)
        self.rear_m = EGO_HALF_LENGTH_M-float(body_shift_m)
        self.memory = MotionMemory()
        self.stopped = False
        self.clear_since = None
        self.clear_samples = set()
        self.people_measurements = {}

    @property
    def stopped(self):
        return self._stopped

    @stopped.setter
    def stopped(self, value):
        # External body/contact stops have no vehicle-only provenance or
        # independently observed clear history. Internal checks update their
        # two cause channels without erasing each other's evidence.
        self._stopped = bool(value)
        self._moving_vehicle_stop_keys = None
        self._independent_stop_pending = bool(value)
        self._independent_clear_since = None
        self._independent_clear_samples = set()

    @staticmethod
    def _moving_stop_key(track, age):
        """Identify fresh measured motion; provisional or inherited motion cannot release a stop."""
        obj = track['object']
        if (len(obj) <= 11 or obj[11] is None or obj[4] != 'npc'
                or 'cycl' in str(obj[3]).lower() or track['state'] != 'moving'
                or not track['confirmed'] or not 0. <= age <= .35):
            return None
        reported = np.asarray(obj[9:11], float)
        velocity = np.asarray(track['velocity'], float)
        try:
            prediction_age = float(obj[15] or 0.) if len(obj) > 15 else 0.
        except (TypeError, ValueError, OverflowError):
            return None
        if (not np.isfinite(reported).all() or not np.isfinite(velocity).all()
                or np.linalg.norm(reported) <= .3 or np.linalg.norm(velocity) <= .3
                or not math.isfinite(prediction_age) or prediction_age != 0.
                or track.get('velocity_stamp', track['stamp']) != track['stamp']):
            return None
        measured = len(obj) > 14 and bool(obj[14])
        semantic = (track.get('source') == 'semantic'
                    and (len(obj) <= 14 or obj[14] is None or bool(obj[14]))
                    and (not str(obj[3]).startswith('fusion_') or track.get('motion_confirmed')))
        if not (measured or semantic):
            return None
        key = (str(obj[3]), obj[11])
        try:
            hash(key)
        except TypeError:
            return None
        return key

    def immediate_stop(self, gap, closing_speed, lag_s, relative_cap):
        """Let ordinary braking reach its speed ceiling; stop on insufficient response.

        The comfortable speed envelope uses brake_mps2. Crossing its ceiling
        calls for deceleration; full stop uses the existing follower/lead hard
        braking limit and subtracts the actual observation/control response.
        """
        room = gap-max(0., closing_speed)*max(0., lag_s)
        return gap <= .1 or (closing_speed > .15 and
            (room <= .1 or closing_speed**2/(2.*room) > EMERGENCY_DECEL_MPS2))

    def check(self, path, ego, batches, now, *, requested_mps,
              raw_clear_m=None, source_age_s=0., observation_ego=None,
              timed_vehicles=False, dynamic_only=False, prediction_horizon_s=None,
              motion_time_scale=1., measured_people=None, released_people=(),
              timed_vehicle_keys=()):
        """Bound speed on this path using footprints and possible departure.

        ``released_people`` are person ids the decision already waited for and
        released (gui release commit): no departure-readiness cap for them; a
        forecast entry into our path still constrains and stops us.

        ``observation_ego`` allows tunnel tracks to stay in wheel/IMU odometry
        while the path remains in the localized vehicle frame used by control.

        ``motion_time_scale`` is the simulator real-time factor (sim_time). The
        memory keeps measured wall-clock velocities and ages, so propagating an
        observation to decision time (velocity x age) is unchanged. The forecast
        runs in simulator seconds, like ego speed and braking: object velocity
        / rtf over the same times as the ego envelope (no second scaling of the
        ego). Motion-evidence gates (moving, possible start, parallel) compare
        the measured speed; wall-second ages remain reaction reserves.

        ``timed_vehicle_keys`` must come from a fresh same-path traffic proof.
        Complete coverage can replace a prior moving-car-only stop without
        delaying for two scans. Every independent current hazard still stops.
        """
        frame = PathFrame(path)
        origin = float(frame.project([[0., 0.]])[0][0])
        remaining_path = max(0., frame.s[-1]-origin)
        cap = max(0., float(requested_mps))
        clear = remaining_path
        speed = abs(float(ego.get('speed_kmh', 0.)))/3.6
        urgent, reason = False, None
        people_emergency = False
        stop_vehicle_keys, independent_stop = set(), False
        covered_restart = False

        def record_object_stop(track, age):
            nonlocal independent_stop
            key = self._moving_stop_key(track, age)
            if key is None:
                independent_stop = True
            else:
                stop_vehicle_keys.add(key)

        checks = []
        if measured_people is not None:
            self.people_measurements = {row['uid']: dict(row, observed_at=now-row['age_s'])
                                        for row in measured_people}
        for source, objects, stamp, age in batches:
            self.memory.update(source, objects, stamp, age, now)
        observed_ego = observation_ego or ego
        heading = math.radians(float(observed_ego['heading_deg']))
        rotation = np.array([[math.cos(heading), -math.sin(heading)],
                             [math.sin(heading), math.cos(heading)]])
        scale = clip_rtf(motion_time_scale)
        # Simulator seconds: the ego's stopping time bounds the forecast.
        horizon = min(4., max(1.2, max(speed, cap)/self.brake_mps2+self.response_s+.5))
        if prediction_horizon_s is not None:
            horizon = max(horizon, float(np.clip(prediction_horizon_s, 1.2, 4.)))
        times = np.arange(0., horizon+.001, .1)
        observations = self.memory.observations(now)
        timed_keys = frozenset(timed_vehicle_keys)
        for track in observations:
            obj = track['object']
            if timed_vehicles and obj[4] == 'npc':
                # The junction's timed OBB checker owns these cars on this
                # same path. A second untimed minimum-distance check revokes
                # clear gaps and stops the ego inside the circulating lane.
                continue
            age = now-track['observed_at']
            certified_static_alias = (obj[3] == 'fusion_parked_vehicle' and obj[4] == 'obstacle'
                and track.get('source') == 'semantic'
                and np.isfinite(np.asarray(obj[9:11], float)).all() and not np.any(obj[9:11]))
            if ((obj[4] == 'npc' or certified_static_alias) and len(obj) > 11
                    and (str(obj[3]), obj[11]) in timed_keys and 0. <= age <= .35):
                # The caller certified this exact body on the current path,
                # including physical contact. Other tracks retain this forecast.
                continue
            velocity = track['velocity']   # measured: metres per wall second
            # Project to decision time once; receipt age and source lag are
            # both included in observed_at by the snapshot provider.
            centre = np.asarray(obj[:2], float)+velocity*age
            measured = self.people_measurements.get(obj[11]) if len(obj)>11 and obj[4]=='pedestrian' else None
            measured_current = bool(measured is not None and measured['stamp'] >= track['stamp'] and
                                    0. <= now-measured['observed_at'] <= .6)
            if measured_current:
                # The camera still owns class and motion history. Associated
                # fresh LiDAR owns position and its response delay; rereading
                # it cannot confirm motion or renew the original observation.
                age = now-measured['observed_at']
                centre = np.asarray(measured['xy'], float)+velocity*age
            local = (centre-[observed_ego['x'], observed_ego['y']]) @ rotation
            measured_local = velocity @ rotation
            local_velocity = measured_local/scale   # simulator units
            length, width = max(.3, float(obj[6])), max(.3, float(obj[7]))
            if max(length, width) > 8.:
                continue  # walls/long structures remain in the raw body check
            # Road semantic people only; tunnel LiDAR movers keep their own rules.
            road_person = ((obj[4] == 'pedestrian' or 'cycl' in str(obj[3]).lower()) and
                           track.get('source') == 'semantic')
            slow_person = road_person and float(np.linalg.norm(velocity)) < PERSON_FORECAST_MIN_MPS
            forecast = local+times[:, None]*(0.*local_velocity if slow_person else local_velocity)
            along, lateral, at = frame.project(forecast)
            along -= origin
            yaw = math.radians(float(obj[8])-float(observed_ego['heading_deg']))
            axis = np.array([math.cos(yaw), math.sin(yaw)])
            tangent = frame.t[at]
            normal = frame.n[at]
            longitudinal = .5*(length*np.abs(tangent@axis)+width*np.abs(normal@axis))
            transverse = .5*(length*np.abs(normal@axis)+width*np.abs(tangent@axis))
            # Motion evidence (noise gates) in measured units; kinematics in
            # simulator units.
            measured_speed = float(np.linalg.norm(velocity))
            moving_speed = measured_speed/scale
            human = obj[4] == 'pedestrian' or 'cycl' in str(obj[3]).lower()
            vehicle = (obj[4] == 'npc' or 'vehicle' in str(obj[3]) or
                       (max(length, width) >= 2.5 and min(length, width) >= 1.))
            # Stations are from the pose: the body spans -rear_m..front_m along the path.
            front_m, rear_m = self.front_m, self.rear_m
            if (vehicle and track['confirmed'] and along[0]+longitudinal[0] < -rear_m and
                    abs(float(measured_local@normal[0])) < .3 and
                    float(local_velocity@tangent[0]) <= speed):
                continue  # a slower car wholly behind cannot block a completed pass
            if dynamic_only and not human and obj[4] != 'npc' and measured_speed < .3:
                # Keep the track available for a later departure. Its present
                # static geometry belongs to the existing bypass/body checker;
                # shape alone must not turn concrete or a known parked-vehicle
                # fragment into an uncertain traffic car with a second margin.
                continue
            # Human/vehicle departure remains possible even after standing was
            # observed. Compact unlabeled clusters retain a smaller uncertainty;
            # known box/cone semantics do not get a fabricated walking speed.
            if vehicle:
                possible_start = not track['confirmed'] or measured_speed < .8
                # Reserve departure during the sensor blind/response interval.
                # Rechecking at 20 Hz supplies the later measured movement; a
                # hypothetical launch seconds later must not close a free lane.
                departure = .75*(age+self.response_s)**2 if possible_start else 0.
                # A stopped vehicle departs along its body heading. Inflating
                # equally in every direction made a parallel parked car occupy
                # an otherwise clear bypass, even while both cars stood still.
                longitudinal += departure*np.abs(tangent@axis)
                uncertainty = (.1+.25*(age+self.response_s)**2+
                               departure*np.abs(normal@axis))
            elif human:
                # Provisional observed walking already owns its forecast;
                # classification confirmation cannot add a second launch.
                possible_start = measured_speed < .8
                uncertainty = (.2+1.8*(age+self.response_s) if possible_start
                               else .1+.15*(times+age))
            else:
                # An unlabeled compact object can depart during the next
                # observation/response interval. Its measured displacement is
                # still forecast above without waiting for a class decision.
                possible_start = True
                uncertainty = .05+.25*(age+self.response_s)**2
            safety_margin = .15 if human or vehicle else .05
            # A standing person's possible departure is a response-time speed
            # constraint below, not a presently measured enlarged body. Treating
            # both as occupancy stopped every near-field bypass at standstill.
            # Measured walking still carries its complete timed forecast; fresh
            # raw returns independently protect the currently occupied corridor.
            body_uncertainty = .1 if human and possible_start else uncertainty
            occupied = np.abs(lateral) <= .946+transverse+safety_margin+body_uncertainty
            ahead = along+longitudinal >= -rear_m
            useful = occupied & ahead & (along-longitudinal < remaining_path+front_m)
            # A possible roadside departure constrains speed before motion is
            # confirmed. Derive a ceiling from the time to the body corridor,
            # rather than expanding the person by today's braking time. The
            # latter alternated clear-at-zero / blocked-after-launch forever.
            toward = -math.copysign(1., float(lateral[0]))*float(local_velocity @ normal[0])
            # Beside or behind our front bumper (true front: EGO_FRONT_M ahead of
            # the rear-axle origin) and not walking into us: braking cannot keep
            # them off our side, and our front can no longer reach them.
            # Motion evidence in measured units (BESIDE_TOWARD_MPS is a walk-in
            # gate: MORAI walkers move >= 1 m/s; kerb jitter stays below it).
            beside = (road_person and along[0]+longitudinal[0]+BESIDE_MARGIN_M <= EGO_FRONT_M and
                      toward*scale < BESIDE_TOWARD_MPS and
                      abs(float(lateral[0]))-transverse[0]-.946 >= BESIDE_CLEARANCE_M)
            # A released person walking back toward our path keeps departure readiness.
            released = (road_person and len(obj) > 11 and obj[11] in released_people and
                        toward*scale < .3)
            if (human and possible_start and not occupied[0] and not beside and not released and
                    -rear_m <= along[0]+longitudinal[0] and along[0] < 25.):
                # A bypass can place a person outside a fixed lateral gate
                # while its stopping time still exceeds the departure time.
                # Let measured lateral room determine the ceiling; a sufficiently
                # wide pass naturally permits cruise without a separate radius.
                lateral_room = float(abs(lateral[0])-.946-transverse[0]-safety_margin-.2)
                # A measured stationary person can accelerate into the lane;
                # treating them as already walking at 1.8 m/s made a clear
                # low-speed turn impossible even after minutes of standing.
                # Keep departure possible with a conservative acceleration
                # envelope. Observed walking still uses the timed forecast
                # below and cannot be cleared by a standing classification.
                # The start is judged from their velocity toward our path: a
                # person walking away (just crossed) must stop before turning
                # back, rather than counting their whole speed toward us.
                response_room = departure_time(lateral_room,
                    toward if road_person else max(0., moving_speed))-age-self.response_s
                departure_cap = self.brake_mps2*max(0., response_room)
                # Outside the measured body corridor, a crawling turn can stop
                # before the person's longitudinal plane. The ordinary 1 m
                # stand-off consumed all of that free space after a crossing,
                # although fresh raw LiDAR still verified the side clearance.
                # Restore it smoothly as speed rises; stale/unassociated people
                # and actual forecast/raw collisions keep the full stop margin.
                stand_off = (.15+.85*min(1., speed/3.) if measured_current else 1.)
                gap = max(0., float(along[0]-longitudinal[0]-front_m-stand_off))
                # It can still stop longitudinally before reaching a far person.
                approach_lag = DEPARTURE_APPROACH_LAG_S if road_person else 0.
                permitted = max(departure_cap, stop_cap(gap, age+self.response_s+approach_lag, self.brake_mps2))
                hard_departure_cap = EMERGENCY_DECEL_MPS2*max(0., response_room)
                must_stop = (speed > hard_departure_cap+.15 and
                    self.immediate_stop(gap, speed, age+self.response_s, permitted))
                if must_stop or permitted < cap:
                    checks.append(dict(name=str(obj[3]), uid=obj[11] if len(obj)>11 else None,
                        state=track['state'], age_s=age, gap_m=gap, cap_mps=permitted,
                        possible_start=True, departure_cap_mps=departure_cap))
                if must_stop:
                    urgent, reason = True, 'live path collision: '+str(obj[3])
                    people_emergency = people_emergency or human
                    record_object_stop(track, age)
                if permitted < cap:
                    cap = permitted
                    reason = reason or 'live path departure readiness: '+str(obj[3])
            if measured_speed >= .3:
                # Compare the two bodies at the same time on the final path.
                # A future crossing behind a departing ego cannot invalidate
                # every bypass merely because all candidates start at origin.
                # Both forecasts already use simulator seconds; the observation
                # delay (wall) is converted for the ego's dead time only.
                lower, upper = travel_envelope(times, speed, cap,
                    (age+self.response_s)*scale, self.brake_mps2)
                useful &= ((along-longitudinal-front_m-1. <= upper) &
                           (along+longitudinal+rear_m+.15 >= lower))
            if not useful.any():
                continue
            initial_gap = float(along[0]-longitudinal[0]-front_m-1.)
            parallel = (vehicle and track['confirmed'] and
                        abs(float(measured_local @ normal[0])) < .3 and
                        float(measured_local @ tangent[0]) > .5)
            if parallel and occupied[0]:
                lead_speed = float(local_velocity @ tangent[0])
                gap = initial_gap
                permitted = lead_speed+stop_cap(gap, age+self.response_s, self.brake_mps2)
                closing = max(0., speed-lead_speed)
                must_stop = self.immediate_stop(gap, closing, age+self.response_s,
                                               permitted-lead_speed)
            else:
                # A possible crossing limits the stopping envelope even when
                # the nominal velocity says that we can pass first.
                gap = max(0., float(np.min((along-longitudinal)[useful]))-front_m-1.)
                permitted = stop_cap(gap, age+self.response_s, self.brake_mps2)
                must_stop = self.immediate_stop(gap, speed, age+self.response_s, permitted)
            checks.append(dict(name=str(obj[3]), uid=obj[11] if len(obj)>11 else None,
                state=track['state'], age_s=age, gap_m=gap, cap_mps=permitted,
                possible_start=bool(not (human or vehicle) or
                    not track['confirmed'] or measured_speed < .8)))
            clear = min(clear, max(0., gap))
            if must_stop:
                urgent, reason = True, 'live path collision: '+str(obj[3])
                people_emergency = people_emergency or human
                record_object_stop(track, age)
            if permitted < cap:
                cap = permitted
                reason = reason or 'live path stopping envelope: '+str(obj[3])
        if raw_clear_m is not None:
            # Raw clearance already includes the vehicle footprint. Preserve
            # the tunnel planner's own side/end margins, including tight passes.
            gap = max(0., float(raw_clear_m)-1.)
            raw_cap = stop_cap(gap, source_age_s+self.response_s, self.brake_mps2)
            if raw_cap < cap:
                cap, reason = raw_cap, reason or 'live lidar stopping envelope'
            if self.immediate_stop(gap, speed, source_age_s+self.response_s, raw_cap):
                urgent, reason = True, reason or 'live lidar collision on published path'
                independent_stop = True
            clear = min(clear, max(0., float(raw_clear_m)))
        token = tuple((source, stamp) for source, _, stamp, _ in batches)
        if independent_stop:
            self._independent_stop_pending = True
        if self._independent_stop_pending:
            fresh = (bool(token) and math.isfinite(float(now)) and
                all(math.isfinite(float(stamp)) and float(stamp) > 0.
                    and math.isfinite(float(age)) and 0. <= float(age) <= .35
                    for _, _, stamp, age in batches) and
                (raw_clear_m is None or (math.isfinite(float(source_age_s))
                                        and 0. <= float(source_age_s) <= .35)))
            if (independent_stop or not fresh or
                    (self._independent_clear_since is not None and
                     now < self._independent_clear_since)):
                self._independent_clear_since = None
                self._independent_clear_samples.clear()
            else:
                if self._independent_clear_since is None:
                    self._independent_clear_since = now
                self._independent_clear_samples.add(token)
                # NPC forecasts may still require a zero command while raw,
                # people and unknown-body checks independently prove clear.
                if (now-self._independent_clear_since >= .1 and
                        len(self._independent_clear_samples) >= 2):
                    self._independent_stop_pending = False
        if urgent:
            self._stopped, self.clear_since = True, None
            self.clear_samples.clear()
            self._moving_vehicle_stop_keys = (frozenset(stop_vehicle_keys)
                if stop_vehicle_keys and not independent_stop else None)
        elif self.stopped:
            prior_keys = self._moving_vehicle_stop_keys
            if (not self._independent_stop_pending and prior_keys and prior_keys <= timed_keys):
                # The caller's fresh same-path map proof replaces only the
                # previous moving-car forecast. This complete check already
                # retained raw, people, static and every uncovered object.
                self.stopped = False
                self.clear_since = None
                self.clear_samples.clear()
                covered_restart = True
            else:
                if self.clear_since is None:
                    self.clear_since = now
                self.clear_samples.add(token)
                # Two different observations, not two timer ticks of one old scan.
                if now-self.clear_since >= .1 and len(self.clear_samples) >= 2:
                    self.stopped = False
                else:
                    urgent, reason = True, 'live path waiting for fresh clear observations'
        cap = 0. if urgent else cap
        return SafetyResult(cap, urgent, clear, reason,
            dict(objects=checks, stopped=urgent, people_emergency=people_emergency,
                 clear_samples=len(self.clear_samples),
                 covered_vehicle_restart=covered_restart,
                 independent_stop_pending=self._independent_stop_pending,
                 independent_clear_samples=len(self._independent_clear_samples),
                 motion_states=[dict(uid=t['object'][11] if len(t['object'])>11 else None,
                                     state=t['state'], stamp=t['stamp'],
                                     **({} if t.get('fusion_mps') is None else
                                        dict(fusion_mps=round(t['fusion_mps'], 2),
                                             onset=t.get('onset') is not None)))
                                for t in observations],
                 people_geometry=[dict(uid=uid, stamp=row['stamp'], age_s=now-row['observed_at'])
                                  for uid, row in self.people_measurements.items()
                                  if 0. <= now-row['observed_at'] <= .6],
                 reaction_s=self.response_s, brake_mps2=self.brake_mps2,
                 source_age_s=source_age_s, raw_clear_m=raw_clear_m,
                 time_scale=round(scale, 3)))
