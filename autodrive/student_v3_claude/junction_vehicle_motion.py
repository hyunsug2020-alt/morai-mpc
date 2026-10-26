"""Fresh map timing ownership for measured moving junction vehicles.

The certificate owns future motion only. Raw returns, people, unassociated
vehicles and actual current body contact retain their independent guards.
Object positions in ``cars`` are already aligned to the current publication;
``batches`` retain source positions and wall-time velocities.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
import hashlib
import math

import numpy as np

from ego_body import EGO_CENTRE_M, EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M
import ring_entry
from sim_time import RTF_MIN, to_sim_objects


POSE_MAX_AGE_S = .2
SOURCE_MAX_AGE_S = .35
CONTACT_MARGIN_M = .1
MOVING_MPS = .3
MATCH_M = 2.
MATCH_HEADING_RAD = math.radians(40.)
# The ring contact model is calibrated for ordinary passenger-car footprints.
# Larger measured vehicles retain the complete generic forecast/body checks.
MAX_MEASURED_LENGTH_M = 6.
MAX_MEASURED_WIDTH_M = 2.4

# Recent source-position fits are shorter than the ordinary two-second ring
# fit. Bridging a detector velocity dropout additionally requires a trusted row.
POSITION_WINDOW_S = .85
POSITION_TRUSTED_AGE_S = .65
POSITION_MIN_SPAN_S = .3
POSITION_MAX_GAP_S = .4
POSITION_MIN_SPEED_MPS = .5
POSITION_RMS_M = .2
POSITION_MAX_RESIDUAL_M = .35


def _position_source(row):
    if (len(row) <= 15 or row[3] != 'lidar_car' or row[4] != 'npc'
            or row[11] is None):
        return False
    values = np.asarray([row[i] for i in (0, 1, 5, 6, 7, 8, 9, 10, 15)], float)
    return bool(np.isfinite(values).all() and values[2] >= 0.
        and .3 <= values[3] <= MAX_MEASURED_LENGTH_M
        and .3 <= values[4] <= MAX_MEASURED_WIDTH_M and values[-1] <= 0.)


def _short_position_fit(samples, stamp):
    """One deterministic latest suffix; a rejected fit never tries another window."""
    recent = [sample[:3] for sample in samples
              if stamp-POSITION_WINDOW_S <= float(sample[0]) <= stamp]
    data = np.asarray(recent, float)
    if (data.ndim != 2 or data.shape[1] != 3 or len(data) < 3
            or not np.isfinite(data).all() or np.any(np.diff(data[:, 0]) <= 1e-6)):
        return None
    first = len(data)-3
    while first > 0 and stamp-data[first, 0] < POSITION_MIN_SPAN_S-1e-6:
        first -= 1
    data = data[first:]
    span, gaps = stamp-data[0, 0], np.diff(data[:, 0])
    if (abs(data[-1, 0]-stamp) > 1e-6 or span < POSITION_MIN_SPAN_S-1e-6
            or np.max(gaps) > POSITION_MAX_GAP_S+1e-6):
        return None
    design = np.column_stack((data[:, 0]-stamp, np.ones(len(data))))
    solution, _, rank, _ = np.linalg.lstsq(design, data[:, 1:3], rcond=None)
    velocity = solution[0]
    speed = float(np.linalg.norm(velocity))
    residual = np.linalg.norm(data[:, 1:3]-design@solution, axis=1)
    if (rank < 2 or not np.isfinite(velocity).all()
            or not POSITION_MIN_SPEED_MPS <= speed <= 45.
            or float(np.sqrt(np.mean(residual**2))) > POSITION_RMS_M
            or float(np.max(residual)) > POSITION_MAX_RESIDUAL_M):
        return None
    segment_velocity = np.diff(data[-3:, 1:3], axis=0)/gaps[-2:, None]
    segment_speed = np.linalg.norm(segment_velocity, axis=1)
    ratios = segment_speed/speed
    if np.any(ratios < .6) or np.any(ratios > 1.5):
        return None
    cosine = segment_velocity@velocity/(segment_speed*speed)
    if np.any(cosine < math.cos(math.radians(30.))):
        return None
    quality = max(ring_entry.FIT_RATIO, float(np.max(ratios)), float(np.max(1./ratios)),
                  1.+float(np.max(residual))/max(.1, speed*span))
    return velocity, speed, quality


def confirm_position_motion(rows, tracker, stamps):
    """Bridge a short velocity dropout using original same-UID PP positions.

    Call with original detector rows, preferably immediately after
    ``RingMotionTracker.update(original_rows, stamps)``. Never supply inherited,
    propagated or previously fitted rows as original source measurements.
    Source stamps and velocities use wall seconds and wall m/s. Global source
    freshness remains the caller's responsibility; this function additionally
    rejects rows more than .35s behind the newest supplied source stamp.

    A recent original measured moving row, its retained history witness and a
    consistent recent position suffix are all required. The deterministic
    suffix is the shortest latest sequence with at least three distinct samples
    spanning .3s; a failed fit is never retried with another window. Neither the
    input rows nor their positions, body yaw, source stamps or history change.
    Tracker diagnostics retain conservative bounds. Repeated reads may reuse
    the same observations but never add samples or renew the trusted stamp.
    Returns ``(tuple_of_rows, frozenset_of_promoted_uids)``. Promotion alone
    grants no map ownership, collision clearance or publication certificate.
    """
    rows = tuple(rows)
    history = getattr(tracker, 'history', {})
    trusted = getattr(tracker, '_junction_position_trusted', None)
    if trusted is None:
        trusted = tracker._junction_position_trusted = {}
    try:
        valid_stamps = [float(stamp) for stamp in stamps.values()
                        if math.isfinite(float(stamp)) and float(stamp) > 0.]
        latest = max(valid_stamps) if valid_stamps else None
    except (TypeError, ValueError, OverflowError):
        return rows, frozenset()
    if latest is None:
        return rows, frozenset()
    for uid in list(trusted):
        if latest-trusted[uid][0] > POSITION_WINDOW_S or trusted[uid][0] > latest:
            trusted.pop(uid, None)
    counts = {}
    for row in rows:
        if len(row) > 11 and row[11] is not None:
            try:
                counts[row[11]] = counts.get(row[11], 0)+1
            except TypeError:
                continue
    out, promoted = [], set()
    for row in rows:
        unchanged = tuple(row)
        out.append(unchanged)
        try:
            if not _position_source(row) or counts.get(row[11]) != 1:
                continue
            uid = row[11]
            stamp = float(stamps[uid])
            if (not math.isfinite(stamp) or stamp <= 0.
                    or not 0. <= latest-stamp <= SOURCE_MAX_AGE_S):
                continue
            samples = list(history.get(uid, ()))
            if samples and float(samples[-1][0]) > stamp+1e-6:
                continue
            current = np.asarray(row[:2], float)
            matching = [sample for sample in samples if abs(float(sample[0])-stamp) <= 1e-6]
            if matching and (len(matching) != 1
                    or np.linalg.norm(np.asarray(matching[0][1:3], float)-current) > 1e-6):
                continue
            velocity = np.asarray(row[9:11], float)
            speed = float(np.linalg.norm(velocity))
            if bool(row[14]):
                # A fitted/inherited True row cannot overwrite a raw False
                # history witness at the same source time.
                original_measured = not matching or (len(matching[0]) > 3
                    and math.isfinite(float(matching[0][3]))
                    and abs(float(matching[0][3])-speed) <= 1e-6)
                if original_measured and speed < POSITION_MIN_SPEED_MPS:
                    # A newer measured standstill ends the dropout bridge.
                    trusted.pop(uid, None)
                    continue
                if (original_measured and speed >= POSITION_MIN_SPEED_MPS
                        and (uid not in trusted or stamp > trusted[uid][0]+1e-6)):
                    trusted[uid] = (stamp, float(row[0]), float(row[1]),
                                   float(row[6]), float(row[7]), *velocity.tolist())
                continue
            # Raw PP uses zero velocity while velocity_valid is False. An
            # inherited nonzero vector is not evidence for a new measurement.
            if speed > 1e-9 or abs(float(row[5])) > 1e-9 or uid not in trusted:
                continue
            previous = trusted[uid]
            trusted_age = stamp-previous[0]
            if not 0. < trusted_age <= POSITION_TRUSTED_AGE_S:
                continue
            witnesses = [sample for sample in samples
                         if abs(float(sample[0])-previous[0]) <= 1e-6]
            if (len(witnesses) != 1 or len(witnesses[0]) < 4
                    or not math.isfinite(float(witnesses[0][3]))
                    or float(witnesses[0][3]) < POSITION_MIN_SPEED_MPS
                    or np.linalg.norm(np.asarray(witnesses[0][1:3], float)-previous[1:3]) > 1e-6):
                continue
            shapes = np.asarray(row[6:8], float)/np.asarray(previous[3:5], float)
            if np.any(shapes < .75) or np.any(shapes > 1.25):
                continue
            prior_velocity = np.asarray(previous[5:7], float)
            innovation = current-np.asarray(previous[1:3])-prior_velocity*trusted_age
            if np.linalg.norm(innovation) > POSITION_MAX_RESIDUAL_M+3.*trusted_age**2:
                continue
            if not matching:
                samples.append((stamp, float(row[0]), float(row[1]), math.nan))
            fitted = _short_position_fit(samples, stamp)
            if fitted is None:
                continue
            fitted_velocity, fitted_speed, quality = fitted
            trusted_cosine = float(prior_velocity@fitted_velocity)/(float(np.linalg.norm(prior_velocity))*fitted_speed)
            if trusted_cosine < math.cos(math.radians(45.)):
                continue
            values = list(row)
            values[5] = fitted_speed
            values[9:11] = fitted_velocity.tolist()
            values[14] = True
            out[-1] = tuple(values)
            promoted.add(uid)
            ratio = float(getattr(tracker, 'ratio', {}).get(uid, ring_entry.FALLBACK_RATIO))
            if not math.isfinite(ratio) or ratio < 1.:
                ratio = ring_entry.FALLBACK_RATIO
            tracker.ratio[uid] = max(ratio, ring_entry.FALLBACK_RATIO, quality)
            tracker.fitted[uid] = fitted_speed
        except (KeyError, TypeError, ValueError, OverflowError, np.linalg.LinAlgError):
            continue
    return tuple(out), frozenset(promoted)


def source_motion_rows(rows, tracker, stamps):
    """Fit moving sources without replacing a current measured standstill.

    The original source rows remain authoritative for valid near-zero velocity.
    Valid moving sources use a consistent recent position fit; invalid detector
    velocities require the separate trusted bridge. Call once per original
    frame; rereading its stamp cannot create a new history sample or refresh a
    trusted observation.
    """
    rows = tuple(rows)
    trusted = getattr(tracker, '_junction_position_trusted', None)
    history = getattr(tracker, 'history', {})
    if trusted:
        # RingMotionTracker.reset() predates this private confirmation state.
        # An absent original witness also handles an individual UID reset.
        for uid, previous in list(trusted.items()):
            witnesses = [sample for sample in history.get(uid, ())
                         if abs(float(sample[0])-previous[0]) <= 1e-6]
            if (len(witnesses) != 1 or len(witnesses[0]) < 4
                    or not np.isfinite(np.asarray(witnesses[0][:4], float)).all()
                    or np.linalg.norm(np.asarray(witnesses[0][1:3], float)-previous[1:3]) > 1e-6
                    or abs(float(witnesses[0][3])-math.hypot(*previous[5:7])) > 1e-6):
                trusted.pop(uid, None)
    fitted = tracker.update(rows, stamps)
    confirmed, promoted = confirm_position_motion(rows, tracker, stamps)
    replacements = {row[11]: row for row in confirmed if len(row) > 11 and row[11] in promoted}
    counts = {}
    for row in rows:
        if len(row) > 11 and row[11] is not None:
            try:
                counts[row[11]] = counts.get(row[11], 0)+1
            except TypeError:
                continue
    try:
        valid_stamps = [float(stamp) for stamp in stamps.values()
                        if math.isfinite(float(stamp)) and float(stamp) > 0.]
    except (TypeError, ValueError, OverflowError):
        valid_stamps = []
    latest = max(valid_stamps) if valid_stamps else None
    out = []
    for original, updated in zip(rows, fitted):
        updated = replacements.get(original[11], updated) if len(original) > 11 else updated
        try:
            # Preserve older source tuples without a prediction-age extension.
            predicted = (len(original) > 15 and original[15] is not None
                         and (not math.isfinite(float(original[15])) or float(original[15]) > 0.))
            measured = (len(original) > 14 and original[3] == 'lidar_car'
                        and original[4] == 'npc' and bool(original[14]) and not predicted)
            velocity = np.asarray(original[9:11], float)
            speed = float(np.linalg.norm(velocity))
            if measured and velocity.shape == (2,) and np.isfinite(velocity).all() and speed < POSITION_MIN_SPEED_MPS:
                updated = original
                uid = original[11]
                getattr(tracker, '_junction_position_trusted', {}).pop(uid, None)
                tracker.fitted[uid] = speed
                ratio = float(tracker.ratio.get(uid, ring_entry.FALLBACK_RATIO))
                tracker.ratio[uid] = max(ring_entry.FALLBACK_RATIO, ratio) if math.isfinite(ratio) else ring_entry.FALLBACK_RATIO
            elif measured and _position_source(original) and counts.get(original[11]) == 1:
                uid = original[11]
                stamp = float(stamps[uid])
                samples = list(tracker.history.get(uid, ()))
                matching = [sample for sample in samples if abs(float(sample[0])-stamp) <= 1e-6]
                if (latest is not None and 0. <= latest-stamp <= SOURCE_MAX_AGE_S
                        and samples and float(samples[-1][0]) <= stamp+1e-6
                        and len(matching) == 1 and math.isfinite(float(matching[0][3]))
                        and abs(float(matching[0][3])-speed) <= 1e-6
                        and np.linalg.norm(np.asarray(matching[0][1:3])-original[:2]) <= 1e-6):
                    recent = _short_position_fit(samples, stamp)
                    if recent is not None:
                        new_velocity, new_speed, quality = recent
                        cosine = float(velocity@new_velocity)/(speed*new_speed)
                        if cosine >= math.cos(math.radians(45.)):
                            robust_speed = float(updated[5])
                            full_history = float(samples[-1][0]-samples[0][0])>=ring_entry.RingMotionTracker.MIN_SPAN_S
                            short_outlier = (full_history and robust_speed>=POSITION_MIN_SPEED_MPS
                                and abs(new_speed-speed)>.3*speed
                                and abs(new_speed-robust_speed)>.3*robust_speed)
                            if not short_outlier:
                                values = list(original)
                                values[5] = new_speed
                                values[9:11] = new_velocity.tolist()
                                updated = tuple(values)
                            # Partial-box centroid shifts can double a .3 s
                            # slope. When both the current measured velocity
                            # and the full source fit disagree, keep the robust
                            # estimate. Actual measured acceleration/deceleration
                            # agreeing with the recent slope still takes effect.
                            chosen_speed = float(updated[5])
                            # Preserve the current raw measurement inside both
                            # speed bounds when its estimate and the fit differ.
                            tracker.ratio[uid] = max(
                                tracker.ratio.get(uid, ring_entry.FALLBACK_RATIO) if short_outlier else quality,
                                speed/chosen_speed, chosen_speed/speed)
                            tracker.fitted[uid] = chosen_speed
        except (IndexError, KeyError, TypeError, ValueError, OverflowError, np.linalg.LinAlgError):
            pass
        # A constant-speed vehicle must not become a larger gap merely because
        # one partial box temporarily underestimates its velocity. Retain the
        # recent measured scalar speed inside the upper source bound. This
        # only widens uncertainty; it grants no movement or source freshness.
        try:
            uid = original[11]
            chosen = float(updated[5])
            history = tracker.history.get(uid, ())
            recent_speeds = [float(sample[3]) for sample in history
                if history[-1][0]-sample[0]<=1.5
                and math.isfinite(float(sample[3])) and sample[3]>=POSITION_MIN_SPEED_MPS]
            if chosen>=POSITION_MIN_SPEED_MPS and len(recent_speeds)>=3:
                tracker.ratio[uid] = max(tracker.ratio.get(uid, ring_entry.FALLBACK_RATIO),
                    float(np.median(recent_speeds))/chosen)
        except (IndexError, TypeError, ValueError, OverflowError):
            pass
        out.append(tuple(updated))
    return tuple(out), promoted


@dataclass(frozen=True)
class JunctionVehicleMotionProof:
    covered_keys: frozenset
    current_clear_m: float
    cap_mps: float
    expires_at: float
    path_digest: str
    ego_frame_digest: str
    created_at: float
    required_rear_travel_m: float = 0.


def _path(path):
    values = np.asarray(path, dtype=np.float64)
    if values.ndim != 2 or values.shape[0] < 3 or values.shape[1] < 2:
        return None
    values = np.ascontiguousarray(values[:, :2])
    if not np.isfinite(values).all() or np.linalg.norm(np.diff(values, axis=0), axis=1).sum() < 8.:
        return None
    return values


def _ego_values(ego):
    values = np.asarray([float(ego[key]) for key in
        ('x', 'y', 'heading_deg', 'speed_kmh', 'stamp')], dtype=np.float64)
    return values if np.isfinite(values).all() and values[4] > 0. else None


def _digest(values):
    return hashlib.sha256(values.tobytes()).hexdigest()[:16]


def proof_matches(proof, ego, world_path, *, now):
    """A certificate cannot transfer to another path, pose, speed or source time."""
    try:
        path, frame = _path(world_path), _ego_values(ego)
        return bool(isinstance(proof, JunctionVehicleMotionProof) and path is not None
            and frame is not None and math.isfinite(float(now))
            and proof.created_at <= float(now) <= proof.expires_at
            and proof.covered_keys and math.isfinite(proof.cap_mps) and proof.cap_mps > 0.
            and math.isfinite(proof.required_rear_travel_m) and proof.required_rear_travel_m >= 0.
            and _digest(path) == proof.path_digest and _digest(frame) == proof.ego_frame_digest)
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def _row(obj, *, measured=False):
    if len(obj) < 12 or obj[4] != 'npc' or obj[11] is None:
        return False
    fields = np.asarray([obj[i] for i in (0, 1, 5, 6, 7, 8, 9, 10)], dtype=float)
    if not np.isfinite(fields).all() or fields[2] < 0. or not .3 <= min(fields[3:5]) <= max(fields[3:5]) <= 8.:
        return False
    hash((str(obj[3]), obj[11]))
    if measured and (len(obj) <= 14 or not obj[14] or
                     (len(obj) > 15 and (not math.isfinite(float(obj[15])) or float(obj[15]) > 0.))):
        return False
    return float(np.linalg.norm(fields[-2:])) > MOVING_MPS


def _key(obj):
    return str(obj[3]), obj[11]


def _aligned(obj, age):
    return np.asarray(obj[:2], float) + np.asarray(obj[9:11], float) * age


def _mapped(predictor, obj):
    """Use directed map matches; the planner's straight fallback is not evidence."""
    velocity = np.asarray(obj[9:11], float)
    heading = math.atan2(velocity[1], velocity[0])
    for xy, arc in predictor.lane_paths(obj[:2], heading, 70.):
        xy, arc = np.asarray(xy, float), np.asarray(arc, float)
        if (xy.ndim != 2 or xy.shape[1] < 2 or len(xy) < 2 or arc.shape != (len(xy),)
                or not np.isfinite(xy[:, :2]).all() or not np.isfinite(arc).all()
                or np.any(np.diff(arc) < 0.)):
            continue
        closest = int(np.argmin(np.linalg.norm(xy[:, :2]-obj[:2], axis=1)))
        if np.linalg.norm(xy[closest, :2]-obj[:2]) > MATCH_M:
            continue
        index = min(closest, len(xy)-2)
        tangent = xy[index+1, :2]-xy[index, :2]
        if np.linalg.norm(tangent) <= 1e-6:
            continue
        angle = abs((math.atan2(tangent[1], tangent[0])-heading+math.pi) % (2.*math.pi)-math.pi)
        if angle <= MATCH_HEADING_RAD:
            return True
    return False


def _separation(ego, obj, position):
    """Signed SAT separation of the actual rear-axle ego and one current OBB."""
    heading = math.radians(float(ego['heading_deg']))
    ef = np.array([math.cos(heading), math.sin(heading)])
    en = np.array([-ef[1], ef[0]])
    yaw = math.radians(float(obj[8]))
    vf = np.array([math.cos(yaw), math.sin(yaw)])
    vn = np.array([-vf[1], vf[0]])
    axes = np.array([ef, en, vf, vn])
    delta = np.asarray(position)-np.asarray([ego['x'], ego['y']], float)-EGO_CENTRE_M*ef
    support = EGO_HALF_LENGTH_M*np.abs(axes@ef)+EGO_HALF_WIDTH_M*np.abs(axes@en)
    support += .5*float(obj[6])*np.abs(axes@vf)+.5*float(obj[7])*np.abs(axes@vn)
    return float(np.max(np.abs(axes@delta)-support))


def _alias(alias, age, measured):
    if alias[3] == 'lidar_car' or not _row(alias):
        return False
    if (len(alias) > 15 and (not math.isfinite(float(alias[15])) or float(alias[15]) > 0.)):
        return False
    ratios = np.asarray(alias[6:8], float)/np.asarray(measured[6:8], float)
    if np.any(ratios < .5) or np.any(ratios > 2.):
        return False
    av, mv = np.asarray(alias[9:11], float), np.asarray(measured[9:11], float)
    cosine = float(av@mv)/(float(np.linalg.norm(av))*float(np.linalg.norm(mv)))
    return bool(cosine >= math.cos(MATCH_HEADING_RAD)
                and np.linalg.norm(_aligned(alias, age)-measured[:2]) <= MATCH_M)


def _parked_camera_row(obj):
    """Only the fusion camera's explicit zero-motion passenger-car label."""
    if (len(obj) <= 15 or obj[3] != 'fusion_parked_vehicle'
            or obj[4] != 'obstacle' or obj[11] is None):
        return False
    values = np.asarray([obj[i] for i in (0, 1, 5, 6, 7, 8, 9, 10, 15)], float)
    return bool(np.isfinite(values).all() and abs(values[2]) <= 1e-9
        and .3 <= min(values[3:5]) <= max(values[3:5]) <= 8.
        and np.linalg.norm(values[6:8]) <= 1e-9 and values[-1] == 0.)


def _parked_alias(alias, age, measured, measured_age):
    """A parked camera duplicate may follow an independently certified PP body."""
    if (not _parked_camera_row(alias) or not _row(measured, measured=True)
            or measured[3] != 'lidar_car' or len(measured) <= 15 or measured[15] != 0.
            or measured[6] > MAX_MEASURED_LENGTH_M or measured[7] > MAX_MEASURED_WIDTH_M
            or not all(math.isfinite(a) and 0. <= a <= SOURCE_MAX_AGE_S
                       for a in (age, measured_age))):
        return False
    ratios = np.asarray(alias[6:8], float)/np.asarray(measured[6:8], float)
    angle = abs((math.radians(float(alias[8])-float(measured[8]))+math.pi/2.) % math.pi-math.pi/2.)
    return bool(np.all((.5 <= ratios) & (ratios <= 2.)) and angle <= MATCH_HEADING_RAD+1e-12
        and np.linalg.norm(np.asarray(alias[:2])-measured[:2]) <= 1.2
        and np.linalg.norm(_aligned(alias, age)-_aligned(measured, measured_age)) <= .5)


def _required_rear_travel(ring):
    """Incoming checks must carry a finite physical rear-clearance distance."""
    if (not isinstance(ring, dict) or ring.get('action') != 'GO'
            or not isinstance(ring.get('checks'), (list, tuple))):
        return None
    distances = {}
    for check in ring['checks']:
        if not isinstance(check, dict) or 'object_id' not in check:
            return None
        value = check.get('required_rear_travel_m')
        if isinstance(value, bool) or value is None:
            return None
        distance = float(value)
        if not math.isfinite(distance) or distance < 0. or check['object_id'] in distances:
            return None
        distances[check['object_id']] = distance
    return distances


def certify_moving_cars(planner, ego, world_path, cars, batches, *, rtf, now,
                        vehicle_age_s, lidar_age_s, requested_mps, excluded_uids=(),
                        profile_ceiling_mps=None, evaluation_out=None):
    """Certify fresh same-path GP GO and explicitly covered measured/alias bodies.

    All ages and ``now`` use wall seconds. Ego speed is native simulator km/h;
    NPC velocities are wall m/s and enter the copied planner converted once.
    A refusal leaves the original generic forecast and static checks intact.
    An independently checked nontraffic profile ceiling may describe the
    future launch; the returned command always remains bounded by requested_mps.
    Optional evaluation_out receives only this call's copied-planner result;
    callers may avoid evaluating identical inputs again after a refusal.
    """
    if evaluation_out is not None:
        evaluation_out.clear()
    try:
        junction = getattr(planner, 'junction', None)
        yield_entry = bool(isinstance(junction, dict) and junction.get('yield_only')
            and junction.get('constant_ring_speed') and not junction.get('following_only')
            and not junction.get('recovery_only') and not planner.junction_committed)
        if not isinstance(junction, dict) or not (junction.get('following_only') or yield_entry):
            return None
        path, frame = _path(world_path), _ego_values(ego)
        if path is None or frame is None:
            return None
        now, rtf, requested = float(now), float(rtf), float(requested_mps)
        profile = requested if profile_ceiling_mps is None else float(profile_ceiling_mps)
        pose_age = float(ego.get('age_s', math.inf))
        if 'received' in ego:
            receipt_age = now-float(ego['received'])
            if not 0. <= receipt_age <= POSE_MAX_AGE_S:
                return None
            pose_age = max(pose_age, receipt_age)
        vehicle_age, lidar_age = float(vehicle_age_s), float(lidar_age_s)
        if (not all(math.isfinite(v) for v in (now, rtf, requested, profile, pose_age, vehicle_age, lidar_age))
                or not RTF_MIN <= rtf <= 1. or min(requested, profile) <= 0.
                or not 0. <= pose_age <= POSE_MAX_AGE_S
                or not 0. <= vehicle_age <= SOURCE_MAX_AGE_S
                or not 0. <= lidar_age <= SOURCE_MAX_AGE_S):
            return None
        excluded = set(excluded_uids)
        for name in ('unknown_ids', 'ghost_ids', 'inherited_ids'):
            excluded.update(junction.get(name) or ())
        cars, batches = list(cars), list(batches)
        predictor = getattr(planner, 'map_predictor', None)
        if predictor is None or not callable(getattr(predictor, 'lane_paths', None)):
            return None
        source_rows = []
        for source, objects, stamp, age in batches:
            stamp, age = float(stamp), float(age)
            source_lag = float(ego['stamp'])-stamp
            if (not math.isfinite(stamp) or stamp <= 0. or not math.isfinite(age)
                    or not -.12 <= source_lag <= SOURCE_MAX_AGE_S
                    or not 0. <= age <= SOURCE_MAX_AGE_S):
                continue
            age = max(age, source_lag)
            for obj in objects:
                source_rows.append((str(source), obj, age))
        measured, alias_references, measured_sources = {}, {}, {}
        for obj in cars:
            if (not _row(obj, measured=True) or obj[11] in excluded
                    or obj[3] != 'lidar_car'
                    or float(obj[6]) > MAX_MEASURED_LENGTH_M
                    or float(obj[7]) > MAX_MEASURED_WIDTH_M
                    or not _mapped(predictor, obj)):
                continue
            observations = [(row, age) for _, row, age in source_rows
                            if len(row) >= 12 and _key(row) == _key(obj) and _row(row, measured=True)]
            if not observations:
                continue
            source_row, source_age = min(observations, key=lambda item: item[1])
            if (float(source_row[6]) > MAX_MEASURED_LENGTH_M
                    or float(source_row[7]) > MAX_MEASURED_WIDTH_M):
                continue
            aligned = _aligned(source_row, source_age)
            if np.linalg.norm(aligned-obj[:2]) > MATCH_M:
                continue
            measured[_key(obj)] = obj
            measured_sources[_key(obj)] = (source_row, source_age)
            reference = list(source_row)
            reference[:2] = aligned
            alias_references[_key(obj)] = tuple(reference)
        if not measured:
            return None
        checked = copy.copy(planner)
        checked.junction = dict(junction)
        checked.speed_ratio = dict(getattr(planner, 'speed_ratio', {}))
        checked._standing_since = dict(getattr(planner, '_standing_since', {}))
        # A copied or overridden evaluator cannot inherit an earlier frame's
        # private authorization, even when its public status still says GO.
        checked._junction_assessment = None
        checked._junction_evaluation_epoch = None
        heading = math.radians(float(ego['heading_deg']))
        rotation = np.array([[math.cos(heading), -math.sin(heading)],
                             [math.sin(heading), math.cos(heading)]])
        local = (path-frame[:2])@rotation
        sim_cars = to_sim_objects(cars, rtf)
        decision = checked.evaluate(sim_cars, dict(ego), local,
                                    profile, now, fresh=True)
        if evaluation_out is not None:
            evaluation_out.update(decision=copy.deepcopy(decision),
                                  status=copy.deepcopy(checked.status))
        cap = float(decision['v_mps']) if decision else 0.
        if (not decision or decision.get('kind') not in ('enter', 'follow')
                or checked.status.get('state') not in
                    (('APPROACH_YIELD',) if yield_entry else ('CRUISE', 'FOLLOW'))
                or checked.status.get('ring', {}).get('action', 'GO') != 'GO'
                or not math.isfinite(cap) or cap <= 0.):
            return None
        required = _required_rear_travel(checked.status.get('ring'))
        if required is None:
            return None
        from traffic_planner import _JunctionAssessment
        assessed = checked._junction_assessment
        # Compare mutable diagnostic output with the immutable evidence made
        # from the actual classified cars and decision in this evaluate call.
        # Missing or edited status must not erase an incoming obligation.
        if (not isinstance(assessed, _JunctionAssessment)
                or assessed.epoch is None or assessed.epoch is not checked._junction_evaluation_epoch
                or required != dict(assessed.incoming)):
            return None
        assessed_ids = assessed.moving_ids
        required_rear_travel = max(required.values(), default=0.)
        measured = {key: obj for key, obj in measured.items() if obj[11] in assessed_ids}
        alias_references = {key: row for key, row in alias_references.items() if key in measured}
        measured_sources = {key: row for key, row in measured_sources.items() if key in measured}
        if not measured:
            return None
        keys = set(measured)
        used = set()
        aliases = []
        for source, obj, age in source_rows:
            if len(obj) < 12 or obj[11] in excluded or obj[4] != 'npc' or obj[3] == 'lidar_car':
                continue
            for key, car in alias_references.items():
                # Association uses the two current source measurements, not
                # the planner's fitted future velocity or a map prediction.
                if _alias(obj, age, car):
                    aliases.append((float(np.linalg.norm(_aligned(obj, age)-car[:2])), source, _key(obj), key))
        for _, source, alias_key, measured_key in sorted(aliases, key=lambda item: (item[0], repr(item[2]))):
            if (source, measured_key) not in used and alias_key not in keys:
                keys.add(alias_key)
                used.add((source, measured_key))
        # Association must be unique among every current PP box, including
        # unknown, stationary and excluded tracks. Those cannot be hidden by
        # selecting only the already certified moving candidates.
        pp_sources = [(obj, age) for _, obj, age in source_rows
                      if len(obj) > 11 and obj[3] == 'lidar_car']
        parked_candidates, target_counts, alias_counts = [], {}, {}
        for source, obj, age in source_rows:
            if not _parked_camera_row(obj) or obj[11] in excluded:
                continue
            neighbours = []
            for pp, pp_age in pp_sources:
                positions = np.asarray([*pp[:2], *pp[9:11]], float)
                if not np.isfinite(positions).all():
                    neighbours = []
                    break
                if (np.linalg.norm(np.asarray(obj[:2])-pp[:2]) <= 1.2
                        or np.linalg.norm(_aligned(obj, age)-_aligned(pp, pp_age)) <= 1.2):
                    neighbours.append(_key(pp))
            if len(neighbours) != 1 or neighbours[0] not in measured_sources:
                continue
            key = neighbours[0]
            pp, pp_age = measured_sources[key]
            if not _parked_alias(obj, age, pp, pp_age):
                continue
            alias_key = _key(obj)
            parked_candidates.append((source, alias_key, key))
            target_counts[(source, key)] = target_counts.get((source, key), 0)+1
            alias_counts[(source, alias_key)] = alias_counts.get((source, alias_key), 0)+1
        parked_keys = set()
        for source, alias_key, key in parked_candidates:
            if (target_counts[(source, key)] == 1 and alias_counts[(source, alias_key)] == 1
                    and (source, key) not in used):
                keys.add(alias_key)
                parked_keys.add(alias_key)
                used.add((source, key))
        clear = math.inf
        expiry = min(now+POSE_MAX_AGE_S-pose_age,
                     now+SOURCE_MAX_AGE_S-max(vehicle_age, lidar_age))
        for key, obj in measured.items():
            clear = min(clear, _separation(ego, obj, obj[:2]))
        for _, obj, age in source_rows:
            if len(obj) >= 12 and _key(obj) in keys:
                if not _row(obj) and not (_key(obj) in parked_keys and _parked_camera_row(obj)):
                    return None
                clear = min(clear, _separation(ego, obj, obj[:2]),
                            _separation(ego, obj, _aligned(obj, age)))
                expiry = min(expiry, now+SOURCE_MAX_AGE_S-age)
        if not math.isfinite(clear) or clear <= CONTACT_MARGIN_M:
            return None
        return JunctionVehicleMotionProof(frozenset(keys), clear, min(requested, cap),
            expiry, _digest(path), _digest(frame), now, required_rear_travel)
    except (AttributeError, IndexError, KeyError, TypeError, ValueError, OverflowError):
        return None
