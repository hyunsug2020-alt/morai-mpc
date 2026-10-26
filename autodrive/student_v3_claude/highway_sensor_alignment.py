"""Align highway sensor geometry with bounded prediction uncertainty."""
import math

import numpy as np

from planning.highway.object_policy import (MISSING_MOTION_ACCEL_MPS2,
                                            MOTION_TRACK_KEEP_S, motion_inferred,
                                            MOTION_ALIGNMENT_MAX_DELTA_S,
                                            PREDICTION_ALIGNMENT_INDEX, PREDICTION_ALIGNMENT_KEY,
                                            prediction_alignment_valid)


MAX_ALIGNMENT_DELTA_S = MOTION_ALIGNMENT_MAX_DELTA_S


def _reject_reused_predictions(cars, info):
    """Invalid timing cannot carry an already aligned forecast into another frame."""
    rejected = []
    for car in cars:
        try:
            reused = (len(car) > PREDICTION_ALIGNMENT_INDEX and
                      isinstance(car[PREDICTION_ALIGNMENT_INDEX], dict) and
                      PREDICTION_ALIGNMENT_KEY in car[PREDICTION_ALIGNMENT_INDEX])
        except (TypeError, IndexError, KeyError):
            reused = False
        if reused:
            row = list(car)
            row[14] = False
            rejected.append(tuple(row))
            info['rejected_count'] += 1
        else:
            rejected.append(car)
    return rejected, info


def align_highway_predictions(cars, source_stamp, target_stamp):
    """Align retained highway forecasts after scan masking has finished.

    Detector positions already include prediction through ``source_stamp``;
    only the remaining source-to-ego interval is applied. Input dimensions
    include highway_objects' missing-motion shell. Positive elapsed time adds
    its uncertainty without shrinking the measured body. Observation age never
    decreases, so rewinding cannot turn a forecast into an observed mask owner.
    A source observation still inside the detector lifetime remains a moving
    forecast through this bounded transport delay, with explicit time lineage
    and the full additional uncertainty. It never becomes a new observation.
    Fresh cars have already been aligned by align_measured_cars and pass through.
    Tracked/assumed flow has a separate propagation and lifetime contract in
    HighwayVehicleMotion; its validity flag is not measured detector velocity.
    """
    cars = list(cars)
    info = dict(timing_valid=False, delta_s=None, aligned_count=0,
                aligned_indices=[], expired_count=0, rejected_count=0,
                reason='invalid sensor timestamp')
    try:
        source, target = float(source_stamp), float(target_stamp)
    except (TypeError, ValueError, OverflowError):
        return _reject_reused_predictions(cars, info)
    if not all(math.isfinite(v) and v > 0. for v in (source, target)):
        return _reject_reused_predictions(cars, info)
    delta = target-source
    info['delta_s'] = delta
    if abs(delta) > MAX_ALIGNMENT_DELTA_S:
        info['reason'] = 'sensor timestamps outside alignment bound'
        return _reject_reused_predictions(cars, info)
    info.update(timing_valid=True, reason='bounded retained prediction alignment')
    aligned = []
    for index, car in enumerate(cars):
        try:
            if (len(car) > PREDICTION_ALIGNMENT_INDEX and
                    car[PREDICTION_ALIGNMENT_INDEX] is not None and
                    not isinstance(car[PREDICTION_ALIGNMENT_INDEX], dict)):
                info['rejected_count'] += 1
                aligned.append(car)
                continue
            # These inputs must be original source-frame predictions. Reusing
            # an aligned output cannot move it twice or extend its lifetime.
            if (len(car) > PREDICTION_ALIGNMENT_INDEX and
                    isinstance(car[PREDICTION_ALIGNMENT_INDEX], dict) and
                    PREDICTION_ALIGNMENT_KEY in car[PREDICTION_ALIGNMENT_INDEX]):
                lineage = car[PREDICTION_ALIGNMENT_INDEX][PREDICTION_ALIGNMENT_KEY]
                try:
                    same_target = (float(lineage['target_stamp']) == target and
                                   prediction_alignment_valid(car, float(car[15])))
                except (TypeError, ValueError, OverflowError, KeyError):
                    same_target = False
                if same_target:
                    aligned.append(car)
                else:
                    # A different target needs the original source observation;
                    # leaving its old age valid would freeze a moving lifetime.
                    row = list(car)
                    row[14] = False
                    aligned.append(tuple(row))
                    info['rejected_count'] += 1
                continue
            age = float(car[15])
            verified = (len(car) > 15 and car[3] == 'lidar_car' and
                car[4] == 'npc' and car[14] == True and
                not motion_inferred(car) and
                0. < age <= MOTION_TRACK_KEEP_S and
                all(math.isfinite(float(car[k])) for k in
                    (0, 1, 2, 5, 6, 7, 8, 9, 10)) and
                min(float(car[6]), float(car[7])) > 0. and float(car[5]) >= 0.)
        except (TypeError, ValueError, OverflowError, IndexError):
            verified = False
        if not verified:
            aligned.append(car)
            continue
        target_age = age+delta
        if target_age < 0.:
            info['rejected_count'] += 1
            aligned.append(car)
            continue
        row = list(car)
        row[15] = max(age, target_age)
        row[0] = float(car[0])+delta*float(car[9])
        row[1] = float(car[1])+delta*float(car[10])
        expansion = MISSING_MOTION_ACCEL_MPS2*max(0., target_age**2-age**2)
        row[6], row[7] = float(car[6])+expansion, float(car[7])+expansion
        row[2] = max(row[6:8])
        if not all(math.isfinite(row[k]) for k in (0, 1, 2, 6, 7, 15)):
            info['rejected_count'] += 1
            aligned.append(car)
            continue
        while len(row) <= PREDICTION_ALIGNMENT_INDEX:
            row.append(None)
        metadata = dict(row[PREDICTION_ALIGNMENT_INDEX] or {})
        metadata[PREDICTION_ALIGNMENT_KEY] = dict(source_stamp=source, target_stamp=target,
            source_age_s=age, target_age_s=row[15])
        row[PREDICTION_ALIGNMENT_INDEX] = metadata
        aligned.append(tuple(row))
        info['aligned_count'] += 1
        info['aligned_indices'].append(index)
    return aligned, info


def align_measured_cars(cars, source_stamp, target_stamp, max_delta_s=.35):
    """Move fresh, verified LiDAR observations to one bounded sensor time.

    Positions are world coordinates and velocity fields 9/10 are world m/s.
    Retained predictions already have their own extrapolation and must not be
    advanced again. A caller needing both scan masks and planning forecasts
    should call this on the original observations for each target timestamp.
    Return new tuples only for aligned observations; preserve other bodies.
    """
    cars = list(cars)
    info = dict(timing_valid=False, delta_s=None, aligned_count=0,
                unchanged_count=len(cars), reason="invalid sensor timestamp")
    try:
        source, target, bound = (float(source_stamp), float(target_stamp),
                                 float(max_delta_s))
    except (TypeError, ValueError, OverflowError):
        return cars, info
    if not all(math.isfinite(v) for v in (source, target, bound)) or min(source, target) <= 0.:
        return cars, info
    delta = target-source
    info["delta_s"] = delta
    if bound < 0. or abs(delta) > min(bound, MAX_ALIGNMENT_DELTA_S):
        info["reason"] = "sensor timestamps outside alignment bound"
        return cars, info
    info.update(timing_valid=True, reason="bounded sensor-time alignment")
    aligned = []
    for car in cars:
        try:
            verified = (len(car) > 15 and car[3] == "lidar_car" and
                        car[14] == True and float(car[15]) == 0. and
                        all(math.isfinite(float(car[k])) for k in (0, 1, 2, 5, 6, 7, 8, 9, 10)) and
                        min(float(car[6]), float(car[7])) > 0. and float(car[5]) >= 0.)
        except (TypeError, ValueError, OverflowError, IndexError):
            verified = False
        if not verified:
            aligned.append(car)
            continue
        row = list(car)
        row[0] = float(car[0])+delta*float(car[9])
        row[1] = float(car[1])+delta*float(car[10])
        # Reject overflow rather than replacing a measured body with NaN/Inf.
        if not math.isfinite(row[0]) or not math.isfinite(row[1]):
            aligned.append(car)
            continue
        aligned.append(tuple(row))
        info["aligned_count"] += 1
    info["unchanged_count"] = len(cars)-info["aligned_count"]
    return aligned, info


def align_moving_observations(objects, source_stamp, target_stamp, max_delta_s=.35):
    """Align fresh LiDAR cars and measured moving fusion NPC/person bodies.

    Fusion obstacle/parked bodies and unknown motion remain unchanged. This is
    suitable for deriving scan-time masks from planning-time observations; it
    does not grant any additional object the right to mask static returns.
    """
    objects = list(objects)
    aligned, info = align_measured_cars(objects, source_stamp, target_stamp, max_delta_s)
    if not info['timing_valid']:
        return aligned, info
    delta = info['delta_s']
    for i, obj in enumerate(objects):
        try:
            measured = (len(obj) > 10 and obj[3] != 'lidar_car' and
                        obj[4] in ('npc', 'pedestrian') and
                        all(math.isfinite(float(obj[k])) for k in (0, 1, 2, 5, 6, 7, 8, 9, 10)) and
                        min(float(obj[6]), float(obj[7])) > 0. and float(obj[5]) >= .5 and
                        math.hypot(float(obj[9]), float(obj[10])) >= .5)
        except (TypeError, ValueError, OverflowError, IndexError):
            measured = False
        if not measured:
            continue
        row = list(obj)
        row[0] = float(obj[0])+delta*float(obj[9])
        row[1] = float(obj[1])+delta*float(obj[10])
        if not math.isfinite(row[0]) or not math.isfinite(row[1]):
            continue
        aligned[i] = tuple(row)
        info['aligned_count'] += 1
    info['unchanged_count'] = len(objects)-info['aligned_count']
    return aligned, info


def align_scan_points(points, scan_ego, ego):
    """Express scan-local XY points in the planning pose's local coordinates.

    This is only a rigid coordinate transform, not object-motion compensation.
    Extra columns are copied unchanged. Invalid poses raise ValueError so the
    caller can retain its existing stale-sensor policy.
    """
    if points is None:
        return None
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] < 2:
        raise ValueError("scan points must have shape (N, 2) or more columns")
    poses = []
    for pose in (scan_ego, ego):
        try:
            values = tuple(float(pose[k]) for k in ("x", "y", "heading_deg"))
        except (TypeError, ValueError, OverflowError, KeyError):
            raise ValueError("sensor alignment requires finite ego poses") from None
        if not all(math.isfinite(v) for v in values):
            raise ValueError("sensor alignment requires finite ego poses")
        poses.append(values)
    (sx, sy, sh), (ex, ey, eh) = poses
    source_yaw, target_yaw = math.radians(sh), math.radians(eh)
    c, s = math.cos(source_yaw), math.sin(source_yaw)
    world_x = sx+c*points[:, 0]-s*points[:, 1]
    world_y = sy+s*points[:, 0]+c*points[:, 1]
    c, s = math.cos(target_yaw), math.sin(target_yaw)
    result = points.copy()
    dx, dy = world_x-ex, world_y-ey
    result[:, 0], result[:, 1] = c*dx+s*dy, -s*dx+c*dy
    return result
