"""Vehicle ownership and static constraints for highway planning."""
import math

import numpy as np


# VehicleTracks retains its original observation and UID for TRACK_KEEP_S=1 s.
# A missing scan is still a moving forecast during that lifetime, not a parked
# body at each extrapolated position. Keep this bound coupled to the detector.
MOTION_TRACK_KEEP_S = 1.
# A retained observation may still be inside the detector's lifetime when its
# source frame arrives. Only explicit source-to-ego alignment may add this
# bounded transport interval; it never renews a detector observation or mask.
MOTION_ALIGNMENT_MAX_DELTA_S = .35
PREDICTION_ALIGNMENT_INDEX = 18
PREDICTION_ALIGNMENT_KEY = 'highway_prediction_alignment'
# Match the existing highway measured-acceleration bound. A missed observation
# can hide braking or steering; grow the forecast body by 1/2*a*age^2 per side.
MISSING_MOTION_ACCEL_MPS2 = 6.
# Camera "obstacle" fragments of a slow measured car (E2E runs 2026-10-09
# 183356Z/185440Z/191227Z: 454 frames within 3 m of a fresh mover) sit on its
# rear face and protrude up to 1.3 m along and 0.8 m across the LiDAR rectangle.
# Owned fragments keep a size bound; a box centred beside or beyond the car
# stays an independent obstacle.
FRAGMENT_CENTRE_ALONG_M = 1.
FRAGMENT_CENTRE_ACROSS_M = .2
FRAGMENT_EXTENT_ALONG_M = 1.5
FRAGMENT_EXTENT_ACROSS_M = .9
FRAGMENT_SIZE_TOLERANCE_M = .5
# Camera "parked vehicle" duplicates of one measured mover: a 9 m truck and a
# 7 m car each kept a second 4.6x1.9 box centred 0.9-1.8 m beyond their front
# face (run follow-20261009T204445Z t=212.3, 503.4, 507.1). The one-to-one match
# gives the car its nearer box; the other stayed a static obstacle in its lane,
# braked the ego to 20 km/h and cancelled two lane changes. Two real bodies
# cannot overlap: a parked box sharing this along-axis depth with a fresh
# mover's rectangle, centred inside its width band, is that mover.
DUPLICATE_OVERLAP_M = .3
# Ego body (local frame origin at the body centre, as in static_clear_distance).
EGO_HALF_LENGTH_M = 2.3175
EGO_HALF_WIDTH_M = .946
# A static camera box reaching this deep into our own body is a projection of
# the ego (E2E runs: a 4.6x1.9 "parked vehicle" 1 m left of our centre at
# 72-84 km/h, raw LiDAR clear, four emergency stops). Raw LiDAR and near-field
# memory keep protecting real contact.
SELF_OVERLAP_M = .3
# Far LiDAR mask for moving cars (only beyond SCAN_MASK_NEAR_M of the ego).
# Unmasked rear-face returns of 20-30 km/h cars ended 0.02-0.63 m beyond the
# 0.4/0.35 m mask and became 30-55 m "static" stop lines.
SCAN_MASK_ALONG_M = .6
SCAN_MASK_ACROSS_M = .3
SCAN_MASK_TIMING_S = .05
SCAN_MASK_PREDICTION_TIMING_S = .15
SCAN_MASK_MAX_PREDICTION_S = .5
SCAN_MASK_NEAR_M = 12.
# Index 16 of a highway vehicle tuple names the source of velocity fields that
# no sensor validated (highway_observations.HighwayVehicleMotion): 'tracked'
# (a LiDAR car measured here along its lane) or 'assumed' (highway flow, also
# for unowned camera parked-vehicle boxes ahead). Such a vehicle is a mover for
# planning (never a standing body, user rule 2026-10-11) but never removes
# LiDAR returns (highway_measured_objects/highway_scan_mask_objects). Measured
# geometry still applies: a fresh LiDAR car owns its own camera fragments and
# parked duplicates, with the narrow (unmeasured) gate in highway_objects.
MOTION_SOURCE_INDEX = 16
INFERRED_MOTION_SOURCES = ('tracked', 'assumed')


def motion_inferred(obj):
    """A highway vehicle whose velocity was tracked/assumed here, not sensor-measured."""
    try:
        return (len(obj) > MOTION_SOURCE_INDEX and isinstance(obj[MOTION_SOURCE_INDEX], str) and
                obj[MOTION_SOURCE_INDEX] in INFERRED_MOTION_SOURCES)
    except TypeError:
        return False


def prediction_alignment_valid(obj, age):
    """Validate original source age and one bounded, nonrenewing alignment."""
    try:
        lineage = obj[PREDICTION_ALIGNMENT_INDEX][PREDICTION_ALIGNMENT_KEY]
        source = float(lineage['source_stamp'])
        target = float(lineage['target_stamp'])
        source_age = float(lineage['source_age_s'])
        target_age = float(lineage['target_age_s'])
        delta = target-source
        return bool(obj[3] == 'lidar_car' and obj[4] == 'npc' and obj[14] == True and
                    not motion_inferred(obj) and
                    all(math.isfinite(v) for v in (source, target, source_age, target_age, age)) and
                    min(source, target) > 0. and 0. < source_age <= MOTION_TRACK_KEEP_S and
                    abs(delta) <= MOTION_ALIGNMENT_MAX_DELTA_S and source_age+delta >= 0. and
                    0. < age <= MOTION_TRACK_KEEP_S+MOTION_ALIGNMENT_MAX_DELTA_S and
                    math.isclose(target_age, max(source_age, source_age+delta), rel_tol=0., abs_tol=1e-9) and
                    math.isclose(age, target_age, rel_tol=0., abs_tol=1e-9))
    except (TypeError, ValueError, OverflowError, IndexError, KeyError):
        return False


def _retained_motion(obj):
    age = float(obj[15]) if len(obj) > 15 else 0.
    speed = float(obj[5])
    age_valid = 0. <= age <= MOTION_TRACK_KEEP_S
    if (len(obj) > PREDICTION_ALIGNMENT_INDEX and
            isinstance(obj[PREDICTION_ALIGNMENT_INDEX], dict) and
            PREDICTION_ALIGNMENT_KEY in obj[PREDICTION_ALIGNMENT_INDEX]):
        age_valid = prediction_alignment_valid(obj, age)
    valid = (len(obj) > 14 and bool(obj[14]) and math.isfinite(speed) and speed >= .5 and
             np.isfinite((*obj[:2], *obj[6:11])).all() and min(obj[6:8]) > 0. and
             np.isfinite(obj[9:11]).all() and np.linalg.norm(obj[9:11]) >= .5 and
             math.isfinite(age) and age_valid)
    return valid, age


def _prediction_footprint(obj):
    if obj[3] != 'lidar_car':
        return obj
    valid, age = _retained_motion(obj)
    if not valid or age == 0.:
        return obj
    row = list(obj)
    expansion = MISSING_MOTION_ACCEL_MPS2*age*age
    row[6], row[7] = float(obj[6])+expansion, float(obj[7])+expansion
    row[2] = max(row[6:8])
    return tuple(row)


def moving_car_owns_obstacle_fragment(fragment, car):
    """Attribute a semantic obstacle fragment on a measured mover to that mover.

    The fragment centre must lie on the observed car rectangle (within the
    camera range/bearing error) and the whole fragment within a bounded halo
    around it; it may not be larger than the car. A box centred beside or
    beyond the car remains an independent obstacle, and predicted, unknown or
    standing cars never acquire this ownership.
    """
    try:
        if (len(fragment) < 9 or fragment[3] != 'fusion_obstacle' or
                fragment[4] != 'obstacle' or len(car) <= 15 or
                car[3] != 'lidar_car' or car[4] != 'npc' or car[14] != True):
            return False
        motion_valid, age = _retained_motion(car)
        if not motion_valid or age != 0.:
            return False
        values = np.asarray([fragment[0], fragment[1], fragment[5],
                             fragment[6], fragment[7], fragment[8]], dtype=float)
        if (not np.isfinite(values).all() or min(values[3:5]) <= 0. or
                not 0. <= values[2] < .5):
            return False
        if len(fragment) > 10 and not np.isfinite(np.asarray(fragment[9:11], float)).all():
            return False
        dimensions = values[3:5]
        body_dimensions = np.asarray(car[6:8], dtype=float)
        if (max(dimensions) > max(body_dimensions)+FRAGMENT_SIZE_TOLERANCE_M or
                min(dimensions) > min(body_dimensions)+FRAGMENT_SIZE_TOLERANCE_M):
            return False
        yaw = math.radians(float(values[5]))
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw), math.cos(yaw)]])
        corners = (np.array([[-1., -1.], [-1., 1.], [1., -1.], [1., 1.]]) *
                   dimensions/2.) @ rotation.T + values[:2]
        yaw = math.radians(float(car[8]))
        body_rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                                  [math.sin(yaw), math.cos(yaw)]])
        centre = (values[:2]-np.asarray(car[:2], dtype=float)) @ body_rotation
        if (abs(centre[0]) > body_dimensions[0]/2.+FRAGMENT_CENTRE_ALONG_M or
                abs(centre[1]) > body_dimensions[1]/2.+FRAGMENT_CENTRE_ACROSS_M):
            return False
        local = (corners-np.asarray(car[:2], dtype=float)) @ body_rotation
        return bool(np.all(np.abs(local[:, 0]) <= body_dimensions[0]/2.+FRAGMENT_EXTENT_ALONG_M+1e-9) and
                    np.all(np.abs(local[:, 1]) <= body_dimensions[1]/2.+FRAGMENT_EXTENT_ACROSS_M+1e-9))
    except (TypeError, ValueError, OverflowError, IndexError):
        return False


def moving_car_owns_parked_duplicate(box, car, delay_s=0.):
    """A camera parked-vehicle box overlapping a fresh measured mover is that mover.

    ``delay_s`` is the camera observation time minus the car's; the car is
    compared where it was when the (unpropagated) parked box was observed.
    Predicted, unknown and standing cars never own it, and a box centred beside
    the car or beyond touching distance stays an independent obstacle.
    """
    try:
        if (len(box) < 9 or box[3] != 'fusion_parked_vehicle' or len(car) <= 15 or
                car[3] != 'lidar_car' or car[4] != 'npc' or car[14] != True):
            return False
        motion_valid, age = _retained_motion(car)
        if not motion_valid or age != 0.:
            return False
        values = np.asarray([box[0], box[1], box[6], box[7], box[8], delay_s], dtype=float)
        if not np.isfinite(values).all() or min(values[2:4]) <= 0. or abs(values[5]) > .35:
            return False
        body = np.asarray(car[6:8], dtype=float)
        if (max(values[2:4]) > max(body)+FRAGMENT_SIZE_TOLERANCE_M or
                min(values[2:4]) > min(body)+FRAGMENT_SIZE_TOLERANCE_M):
            return False
        yaw = math.radians(float(car[8]))
        axis = np.array([math.cos(yaw), math.sin(yaw)])
        delta = values[:2]-np.asarray(car[:2], dtype=float)-values[5]*np.asarray(car[9:11], dtype=float)
        along, across = float(delta @ axis), float(delta @ [-axis[1], axis[0]])
        relative = math.radians(values[4])-yaw
        extent = values[2]*abs(math.cos(relative))+values[3]*abs(math.sin(relative))
        return bool(abs(across) <= body[1]/2.+FRAGMENT_CENTRE_ACROSS_M and
                    body[0]/2.+extent/2.-abs(along) >= DUPLICATE_OVERLAP_M)
    except (TypeError, ValueError, OverflowError, IndexError):
        return False


def ego_body_penetration_m(obj, ego):
    """Depth (m) by which a box reaches into the ego body rectangle; <= 0 when apart."""
    try:
        values = np.asarray([obj[0], obj[1], obj[6], obj[7], obj[8],
                             ego['x'], ego['y'], ego['heading_deg']], dtype=float)
    except (TypeError, ValueError, KeyError, IndexError, OverflowError):
        return 0.
    if not np.isfinite(values).all() or min(values[2:4]) <= 0.:
        return 0.
    heading = math.radians(values[7])
    ego_x = np.array([math.cos(heading), math.sin(heading)])
    ego_y = np.array([-ego_x[1], ego_x[0]])
    yaw = math.radians(values[4])
    obj_u = np.array([math.cos(yaw), math.sin(yaw)])
    obj_v = np.array([-obj_u[1], obj_u[0]])
    delta = values[:2]-values[5:7]
    depth = float('inf')
    for axis in (ego_x, ego_y, obj_u, obj_v):
        reach = (EGO_HALF_LENGTH_M*abs(float(ego_x @ axis)) + EGO_HALF_WIDTH_M*abs(float(ego_y @ axis)) +
                 values[2]/2.*abs(float(obj_u @ axis)) + values[3]/2.*abs(float(obj_v @ axis)))
        depth = min(depth, reach-abs(float(delta @ axis)))
    return depth


def _static_camera_box(obj):
    try:
        return obj[3] != 'lidar_car' and obj[4] != 'pedestrian' and not float(obj[5]) >= .5
    except (TypeError, ValueError, IndexError):
        return False


def _moving_semantic_npc(obj):
    """An observed semantic vehicle with finite, independently measured motion."""
    try:
        return bool(obj[4] == 'npc' and len(obj) > 10 and not motion_inferred(obj) and
                    (len(obj) <= 14 or obj[14] == True or
                     (obj[14] is None and obj[3] == 'fusion_npc' and
                      (len(obj) <= 15 or obj[15] == 0.))) and
                    np.isfinite((*obj[:2], *obj[5:11])).all() and min(obj[6:8]) > 0. and
                    float(obj[5]) >= .5 and math.hypot(float(obj[9]), float(obj[10])) >= .5)
    except (TypeError, ValueError, OverflowError, IndexError):
        return False


def highway_objects(fused, cars, ego=None, parked_delay_s=0.):
    """Let a LiDAR vehicle own its matching semantic vehicle footprint.

    A camera track without confirmed motion is published as a parked obstacle.
    Retaining it beside the matching moving vehicle creates a second, stationary
    collision box and engages static braking. A measured moving semantic NPC
    stays beside an unknown/inferred LiDAR car so its motion and body remain
    available to scan masking and traffic checks. Unmatched obstacles remain intact,
    except a static camera box inside our own body (given ``ego``), which is a
    projection of the ego itself, and a parked duplicate overlapping a fresh
    mover (``parked_delay_s``: camera minus car observation time).
    """
    def same_vehicle(semantic, car):
        measured = (len(car) > 14 and car[14] and (len(car) <= 15 or car[15] <= 0.) and
                    not motion_inferred(car))
        # A current semantic NPC has independently measured world motion. An
        # unknown/assumed LiDAR velocity cannot replace that measurement: the
        # inferred car cannot mask its returns, and may be faster than this
        # measured leader. Keep both bodies for the existing timed checks.
        if _moving_semantic_npc(semantic) and not measured:
            return False
        delta = np.asarray(semantic[:2])-np.asarray(car[:2])
        yaw = math.radians(float(car[8]))
        along = abs(float(delta @ [math.cos(yaw), math.sin(yaw)]))
        across = abs(float(delta @ [-math.sin(yaw), math.cos(yaw)]))
        # A semantic centroid briefly moved 1.4 m sideways and 4 m along the
        # road at the highway exit
        # and became a second "crossing car" beside its measured LiDAR track.
        # Allow bounded camera error only around a currently observed car with
        # confirmed motion. Predicted/missing tracks cannot own that observation.
        along_limit = max(float(car[6]), float(semantic[6]))*.5
        across_limit = max(float(car[7]), float(semantic[7]))*.5
        if measured:
            along_limit = min(4.5, along_limit + 2.)
            across_limit = min(1.9, across_limit + .75)
        return along <= along_limit and across <= across_limit
    fused, cars = list(fused), list(cars)
    for i, obj in enumerate(fused):
        # ObjectReceiver pads semantic tuples with None at the LiDAR-specific
        # validity slot. Preserve measured semantic velocity through the common
        # highway motion contract; never promote an explicit invalid flag or a
        # predicted/stationary body. Input tuples and source ages stay intact.
        if (len(obj) > 14 and obj[3] == 'fusion_npc' and obj[14] is None and
                (len(obj) <= 15 or obj[15] == 0.) and _moving_semantic_npc(obj)):
            row = list(obj)
            row[14] = True
            fused[i] = tuple(row)
    matches = [(float(np.linalg.norm(np.asarray(o[:2])-np.asarray(c[:2]))), i, j)
               for i, o in enumerate(fused) if o[4] == "npc" or o[3] == "fusion_parked_vehicle"
               for j, c in enumerate(cars) if (len(c) <= 15 or c[15] <= 0.) and same_vehicle(o, c)]
    semantic_owned = {i for i, obj in enumerate(fused)
                      if any(moving_car_owns_obstacle_fragment(obj, car) for car in cars)}
    car_owned = set()
    for _, i, j in sorted(matches):
        if i not in semantic_owned and j not in car_owned:
            semantic_owned.add(i)
            car_owned.add(j)
    semantic_owned |= {i for i, obj in enumerate(fused) if i not in semantic_owned and
                       any(moving_car_owns_parked_duplicate(obj, car, parked_delay_s) for car in cars)}
    if ego is not None:
        semantic_owned |= {i for i, obj in enumerate(fused)
                           if _static_camera_box(obj) and
                           ego_body_penetration_m(obj, ego) > SELF_OVERLAP_M}
    return ([o for i, o in enumerate(fused) if i not in semantic_owned] +
            [_prediction_footprint(car) for car in cars])


def highway_tentative_objects(tentative, cars, ego=None, parked_delay_s=0.):
    """Unconfirmed camera boxes keep their cautious slowdown unless they are a
    measured mover's own fragment/duplicate or a projection of the ego body."""
    cars = list(cars)
    return [obj for obj in tentative
            if not any(moving_car_owns_obstacle_fragment(obj, car) or
                       moving_car_owns_parked_duplicate(obj, car, parked_delay_s) for car in cars) and
            not (ego is not None and _static_camera_box(obj) and
                 ego_body_penetration_m(obj, ego) > SELF_OVERLAP_M)]


def highway_static_objects(objects):
    """Keep unknown/stopped/expired bodies static; retain tracked mover forecasts.

    On the highway, cars ahead without a detector velocity arrive here with
    tracked/assumed forward motion (HighwayVehicleMotion) and are movers.
    """
    result = []
    for obj in objects:
        stopped = not math.isfinite(float(obj[5])) or float(obj[5]) < .5
        if obj[3] == "lidar_car":
            valid_motion, _ = _retained_motion(obj)
            stopped = stopped or not valid_motion
        if stopped:
            result.append(obj)
    return result

def highway_measured_objects(objects):
    """Only observed vehicles may remove measured static returns or memory.

    A body with tracked/assumed (not measured) motion keeps its returns: the raw
    LiDAR body check and far-return persistence still see it.
    """
    return [o for o in objects if not motion_inferred(o) and (o[3] != "lidar_car" or
            (_retained_motion(o)[0] and _retained_motion(o)[1] == 0.))]


def highway_scan_mask_objects(objects):
    """Moving-car rectangles that remove their own returns beyond the near field.

    Starts from the same measured movers as ``highway_measured_objects`` and adds
    a bounded timing/shape halo, plus recent predictions (<= 0.5 s) at their
    nominal size. Standing, unknown and expired bodies never mask returns, and
    the near field keeps the unexpanded measured mask (see SCAN_MASK_NEAR_M).
    """
    result = []
    for obj in objects:
        if obj[3] != 'lidar_car' or motion_inferred(obj):
            continue
        valid, age = _retained_motion(obj)
        if not valid or age > SCAN_MASK_MAX_PREDICTION_S:
            continue
        expansion = MISSING_MOTION_ACCEL_MPS2*age*age
        length, width = float(obj[6])-expansion, float(obj[7])-expansion
        if min(length, width) <= 0.:
            continue
        speed = float(obj[5])
        along = (SCAN_MASK_ALONG_M+SCAN_MASK_TIMING_S*speed if age == 0. else
                 SCAN_MASK_ALONG_M+SCAN_MASK_PREDICTION_TIMING_S*speed+age*(1.+1.5*age))
        across = SCAN_MASK_ACROSS_M+.5*age
        row = list(obj)
        row[6], row[7] = length+2.*along, width+2.*across
        row[2] = max(row[6:8])
        result.append(tuple(row))
    return result


def without_far_moving_returns(points, mask_objects, ego, near_m=SCAN_MASK_NEAR_M):
    """Remove returns inside scan-time mover rectangles beyond near_m only."""
    from perception.pedestrian_motion import without_moving_objects
    if points is None or not len(points) or not mask_objects:
        return points
    points = np.asarray(points)
    far = np.hypot(points[:, 0], points[:, 1]) > near_m
    if not far.any():
        return points
    return np.vstack((points[~far], without_moving_objects(points[far], mask_objects, ego)))
