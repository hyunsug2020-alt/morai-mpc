"""Moving lane changes with measured rear traffic and retained near-field obstacles."""
import json
import math
import copy
from dataclasses import replace
import os
import time
from pathlib import Path

import numpy as np
from traffic_planner import vehicle_motion_heading
from highway_speed_plan import (HighwaySpeedPlanner, TRAFFIC_CONTROL_HORIZON_S, CONTROL_LOOKAHEAD_S,
                                HEADWAY_S, MIN_GAP_M, GAP_GAIN, _cached_command_preview)
from sim_time import clip_rtf
from ego_body import EGO_CENTRE_M, EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M
from planning.highway.trajectory_geometry import normalize_path, sample_pose, projected_halves


def projected_support(obj, yaw_rad):
    return projected_halves(max(.3, float(obj[6])), max(.3, float(obj[7])),
                            math.radians(float(obj[8])) - yaw_rad)


def local_points(points, ego):
    yaw = math.radians(ego["heading_deg"])
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    return (np.asarray(points) - [ego["x"], ego["y"]]) @ rotation


def world_points(points, ego):
    yaw = math.radians(ego["heading_deg"])
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    return np.asarray(points) @ rotation.T + [ego["x"], ego["y"]]


def path_curvature(path):
    path = np.asarray(path)
    a, b = np.diff(path, axis=0)[:-1], np.diff(path, axis=0)[1:]
    chord = a + b
    return 2 * np.abs(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]) / np.maximum(
        1e-6, np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1) * np.linalg.norm(chord, axis=1))


# Highway curve lateral acceleration. 2.0 made curvature the main highway
# limit (27 s of 43 s, down to 24 km/h, live 2026-10-03). 3.5 held the on-ramp
# curve (r 45-65 m) at 44-46 km/h while the follower tracked 3.4 m/s² with
# 3 cm CTE (follow-20261010T140051Z mpc.csv); 4.0 stays below the follower's
# own curve_alat_max 4.2 (2026-10-10, aggressive highway).
HIGHWAY_CURVE_ALAT_MPS2 = float(os.environ.get("STUDENT_HIGHWAY_CURVE_ALAT", "4.0"))
LANE_ADMISSION_MARGIN_M = .6
CURRENT_PATH_FOOTPRINT_MARGIN_M = .2


def geometry_speed_cap(path, cruise_mps, lateral_accel=None, decel=3.):
    lateral_accel = HIGHWAY_CURVE_ALAT_MPS2 if lateral_accel is None else lateral_accel
    path = np.asarray(path, dtype=float)[:, :2]
    if len(path) < 4:
        return 0.
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    # Curvature over 3 m chords: from 0.25-0.5 m neighbours 5 mm of point jitter
    # read as a 15 m radius and capped a straight lane change at 25-49 km/h
    # (auto runs 135/136; synthetic straight path: 90 -> 10 km/h).
    if arc[-1] > 9.:
        stations = np.arange(0., arc[-1] + 1e-6, 3.)
        path = np.column_stack([np.interp(stations, arc, path[:, i]) for i in (0, 1)])
        arc = stations
    curvature = path_curvature(path)
    curve_limit_sq = lateral_accel/np.maximum(.0001, curvature)
    reserve = max(8., cruise_mps*.5)
    limit_sq = curve_limit_sq + 2*decel*np.maximum(0., arc[1:-1]-reserve)
    return min(cruise_mps, math.sqrt(max(0., float(limit_sq.min()))))


def geometry_speed_profile(path, cruise_mps, lateral_accel=None, decel=3.):
    """geometry_speed_cap as a speed envelope along the path: (stations, speeds).

    speeds[0] equals geometry_speed_cap(path, cruise_mps); each later station
    only keeps the curvature still ahead of it. A forecast that applies the
    current-position cap for 14 s held a 3.6 s lane change at its peak-curvature
    speed long after the change ended, so slower long changes won on progress.
    """
    lateral_accel = HIGHWAY_CURVE_ALAT_MPS2 if lateral_accel is None else lateral_accel
    path = np.asarray(path, dtype=float)[:, :2]
    if len(path) < 4:
        return np.array([0.]), np.array([0.])
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    if arc[-1] > 9.:
        stations = np.arange(0., arc[-1] + 1e-6, 3.)
        path = np.column_stack([np.interp(stations, arc, path[:, i]) for i in (0, 1)])
        arc = stations
    curve_limit_sq = lateral_accel/np.maximum(.0001, path_curvature(path))
    points = arc[1:-1]
    reserve = max(8., cruise_mps*.5)
    stations = np.r_[0., points]
    ahead = points[None, :] >= stations[:, None]-1e-9
    limit_sq = curve_limit_sq[None, :] + 2*decel*np.maximum(0., points[None, :]-stations[:, None]-reserve)
    limit_sq = np.where(ahead, limit_sq, np.inf).min(axis=1)
    speeds = np.minimum(cruise_mps, np.sqrt(np.maximum(0., limit_sq)))
    return np.r_[stations, arc[-1]], np.r_[speeds, cruise_mps]


def lane_change_path(route, target_offset, transition_m, match_heading=False, heading_span_m=35.):
    """Quintic lateral transition, joined at the measured current lane offset.

    ``heading_span_m`` bounds the distance over which the measured heading is
    turned into the route direction (shorter: quicker, sharper correction)."""
    route = np.asarray(route, dtype=float)[:, :2]
    if match_heading and len(route) > 1 and route[0, 0] < 0.:
        ahead = np.flatnonzero(route[:, 0] >= 0.)
        if not len(ahead) or len(route)-ahead[0] < 2:
            raise ValueError('highway join has no forward suffix')
        route = route[ahead[0]:]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    tangent = np.gradient(route, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-6)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    start_offset = -float(np.dot(route[0], normal[0]))
    u = np.clip(arc / transition_m, 0., 1.)
    smooth = u**3 * (10. - 15.*u + 6.*u*u)
    offset = start_offset + (target_offset-start_offset)*smooth
    if match_heading:
        # In the ego frame the measured initial direction is +x. Preserve it
        # when a new checked plan replaces a partly completed lane change.
        slope = -tangent[0, 1] / max(.1, tangent[0, 0])
        heading_span = min(transition_m, heading_span_m)
        heading_u = np.clip(arc / heading_span, 0., 1.)
        offset += slope * heading_span * (heading_u - 6.*heading_u**3 +
                                          8.*heading_u**4 - 3.*heading_u**5)
    path = route + normal*offset[:, None]
    if match_heading:
        # The nearest native vertex is an along-route projection, not the
        # measured axle. A matched control join starts at that measured pose.
        path -= path[0]*(1.-smooth)[:, None]
    return path


def _dot_xy(first, second):
    """Two-axis projection with the same addition order as NumPy's row sum."""
    return first[:, 0]*second[:, 0] + first[:, 1]*second[:, 1]


class _GapForecastContext:
    """Immutable geometry and measured motion shared by one candidate's probes.

    Preparation is lazy so invalid-input checks retain their original order.
    Only the known highway model is snapshotted and forecast-cached; arbitrary
    providers retain their own calling and state semantics.
    """
    def __init__(self, positions, yaws):
        self.path = None
        self.angles = None
        self.positions, self.yaws = positions, yaws
        self.forecasts = {}
        self.path_relations = {}
        self.snapshot_ready = False
        self.cache_forecasts = positions is None and yaws is None

    @staticmethod
    def frozen(value):
        value = np.array(value, copy=True)
        value.flags.writeable = False
        return value

    def normalized(self, path):
        if self.path is None:
            path, stations = normalize_path(path)
            self.path, self.stations = self.frozen(path), self.frozen(stations)
        return self.path, self.stations

    def geometry(self, path, stations, ego):
        if self.angles is None:
            tangent = np.gradient(path, stations, axis=0)
            self.angles = self.frozen(np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0])))
            self.world_path = self.frozen(world_points(path, ego))
        if not self.snapshot_ready:
            self.snapshot_ready = True
            owner = getattr(self.positions, '__self__', None)
            if (isinstance(owner, HighwaySpeedPlanner) and
                    getattr(self.yaws, '__self__', None) is owner and
                    getattr(self.positions, '__func__', None) is HighwaySpeedPlanner.object_positions and
                    getattr(self.yaws, '__func__', None) is HighwaySpeedPlanner.object_yaws):
                snapshot = copy.copy(owner)
                snapshot.motion_paths = {key: (self.frozen(value[0]), self.frozen(value[1]), *value[2:])
                                         for key, value in owner.motion_paths.items()}
                snapshot.stable_yaws = dict(owner.stable_yaws)
                self.positions, self.yaws = snapshot.object_positions, snapshot.object_yaws
                self.cache_forecasts = True
        return self.angles, self.world_path

    def forecast(self, obj, times):
        key = (id(obj), times.dtype.str, times.shape, times.tobytes())
        previous = self.forecasts.get(key) if self.cache_forecasts else None
        if previous is not None and previous[0] is obj:
            return previous[1]
        velocity = [float(obj[9]), float(obj[10])] if len(obj) > 10 else (0., 0.)
        other = (self.positions(obj, times) if self.positions is not None else
                 np.asarray(obj[:2]) + times[:, None]*velocity)
        angle = (self.yaws(obj, times) if self.yaws is not None else
                 np.full(len(times), math.radians(float(obj[8]))))
        other, angle = self.frozen(other), self.frozen(angle)
        # Validate in timed_gap before using these arrays to form box axes.
        result = other, angle
        if self.cache_forecasts:
            self.forecasts[key] = obj, result
        return result

    def path_relation(self, obj):
        previous = self.path_relations.get(id(obj))
        if previous is not None and previous[0] is obj:
            return previous[1]
        at = int(np.argmin(np.linalg.norm(self.world_path-np.asarray(obj[:2]), axis=1)))
        tangent = self.world_path[min(at+1, len(self.world_path)-1)]-self.world_path[max(at-1, 0)]
        tangent /= max(1e-6, float(np.linalg.norm(tangent)))
        normal = np.array([-tangent[1], tangent[0]])
        offset = float((np.asarray(obj[:2])-self.world_path[at]) @ normal)
        vx, vy = (float(obj[9]), float(obj[10])) if len(obj) > 10 else (0., 0.)
        drift = -math.copysign(1., offset)*float(np.asarray([vx, vy]) @ normal)
        result = offset, drift
        self.path_relations[id(obj)] = obj, result
        return result


def timed_gap(path, ego, objects, target_mps, horizon_s=7.0, lane_change=True, zipper=False,
              speed_trajectory=None, object_positions=None, object_yaws=None,
              ignore_rear_approach=False, footprint_margin_m=LANE_ADMISSION_MARGIN_M,
              body_shift_m=EGO_CENTRE_M, _prepared=None):
    """Check simultaneous footprints and closing rear gaps during a lane change."""
    try:
        path, arc = normalize_path(path) if _prepared is None else _prepared.normalized(path)
    except (ValueError, TypeError):
        return {"when": 0., "reason": "invalid highway path"}
    try:
        valid_pose = all(math.isfinite(float(ego[k])) for k in ('x', 'y', 'heading_deg'))
    except (KeyError, TypeError, ValueError):
        valid_pose = False
    if not valid_pose:
        return {"when": 0., "reason": "invalid highway pose"}
    try:
        body_shift_m, footprint_margin_m = float(body_shift_m), float(footprint_margin_m)
        valid_body = (math.isfinite(body_shift_m) and abs(body_shift_m) <= EGO_HALF_LENGTH_M and
                      math.isfinite(footprint_margin_m) and footprint_margin_m >= 0.)
    except (ValueError, TypeError):
        valid_body = False
    if not valid_body:
        return {"when": 0., "reason": "invalid highway body geometry"}
    if speed_trajectory is not None:
        times = speed_trajectory.times
        distances = speed_trajectory.distances
    else:
        times = np.arange(0., horizon_s + .01, .1)
        velocity = max(0., float(ego.get("speed_kmh", 0.))) / 3.6
        distances, distance = [0.], 0.
        for _ in times[1:]:
            next_v = velocity + np.clip(target_mps-velocity, -.12, .12)
            distance += (velocity+next_v)*.05
            distances.append(distance)
            velocity = next_v
        distances = np.asarray(distances)
    times, distances = np.asarray(times), np.asarray(distances)
    if (times.ndim != 1 or distances.ndim != 1 or len(times) < 2 or times.shape != distances.shape or
            not np.isfinite(times).all() or not np.isfinite(distances).all() or
            np.any(np.diff(times) <= 0.) or np.any(np.diff(distances) < -1e-9) or
            distances[0] != 0. or times[0] != 0.):
        return {"when": 0., "reason": "invalid highway trajectory"}
    front_m = EGO_HALF_LENGTH_M + body_shift_m
    if distances[-1] + max(3., front_m + footprint_margin_m) > arc[-1]:
        return {"when": 0., "reason": "lane-change path too short"}
    path_angles, world_path = (None, None) if _prepared is None else _prepared.geometry(path, arc, ego)
    xy, yaw = sample_pose(path, arc, distances, ego["heading_deg"], path_angles)
    world = world_points(xy, ego)
    if world_path is None:
        world_path = world_points(path, ego)
    et = np.column_stack((np.cos(yaw), np.sin(yaw)))
    en = np.column_stack((-et[:, 1], et[:, 0]))
    body_world = world + body_shift_m*et
    earliest = None
    for obj in objects:
        if obj[4] != "npc" and float(obj[5]) < .5:
            # Stationary obstacles are checked by the static body clearance.
            # Here the 0.6 m timing shell turned roadside cones 25 cm from the
            # body into "imminent predicted collision" stops (2026-10-03).
            continue
        if (obj[4] == "npc" and len(obj) > 14 and not obj[14] and
                float((np.asarray(obj[:2]) - world[0]) @ et[0]) > 30.):
            # A car first seen 50 m ahead has no speed yet; read as standing it made
            # "predicted crossing" dips to our own speed every 1-2 s, and the 2.5 m/s^2
            # ramp never recovered (highway entry held at 37 km/h, auto run 143). A
            # truly standing car is braked for by the static stopping reserve.
            continue
        vx, vy = (float(obj[9]), float(obj[10])) if len(obj) > 10 else (0., 0.)
        positions_provider = object_positions if _prepared is None else _prepared.positions
        native_motion = (positions_provider is not None and
                         (obj[11] if len(obj)>11 else id(obj)) in
                         getattr(getattr(positions_provider, '__self__', None), 'motion_paths', {}))
        if _prepared is None:
            other = (object_positions(obj, times) if object_positions is not None else
                     np.asarray(obj[:2]) + times[:, None]*[vx, vy])
            angle = (object_yaws(obj, times) if object_yaws is not None else
                     np.full(len(times), math.radians(float(obj[8]))))
        else:
            other, angle = _prepared.forecast(obj, times)
        other, angle = np.asarray(other), np.asarray(angle)
        if (other.shape != world.shape or angle.shape != times.shape or
                not np.isfinite(other).all() or not np.isfinite(angle).all()):
            return {"when": 0., "reason": "invalid highway object forecast"}
        ot = np.column_stack((np.cos(angle), np.sin(angle)))
        on = np.column_stack((-ot[:, 1], ot[:, 0]))
        if (not np.isfinite(np.asarray(obj[6:8], dtype=float)).all() or
                min(float(obj[6]), float(obj[7])) <= 0.):
            return {"when": 0., "reason": "invalid highway object forecast"}
        length, width = max(.3, float(obj[6])), max(.3, float(obj[7]))
        delta = other-body_world
        hit = np.ones(len(times), dtype=bool)
        physical_hit = np.ones(len(times), dtype=bool)
        # Lane admission reserves more lateral room than execution of an
        # already checked path. Keep actual body overlap separately observable.
        for axis in (et, en, np.broadcast_to(ot, et.shape), np.broadcast_to(on, en.shape)):
            radius = (EGO_HALF_LENGTH_M*np.abs(_dot_xy(et, axis)) + EGO_HALF_WIDTH_M*np.abs(_dot_xy(en, axis)) +
                      length/2*np.abs(_dot_xy(ot, axis)) + width/2*np.abs(_dot_xy(on, axis)))
            projection = np.abs(_dot_xy(delta, axis))
            physical_hit &= projection <= radius
            hit &= projection <= radius+footprint_margin_m
            if not hit.any():
                # A separating axis at every sampled instant proves both
                # shells empty (physical_hit is a subset for margin >= 0).
                # Rear headway checks below still run for this object.
                break
        footprint_hit = hit.copy()
        longitudinal = _dot_xy(delta, et)
        # A rear/side uncertainty shell can be escaped by driving on. Actual
        # present or future body contact must survive every exception.
        beside_offset = abs(float((delta[0]*en[0]).sum()))
        if not native_motion and not physical_hit.any() and (
                (longitudinal[0] < -2. and beside_offset > 1.) or longitudinal[0] < -1.):
            # Beside/behind and not touching: stopping cannot avoid it, only
            # driving on can (live 2026-10-03 23:55: parked 30 s mid lane
            # change with a car at the left rear, "imminent collision" at t=0).
            footprint_hit[:] = False
        lateral = np.abs(_dot_xy(delta, en))
        motion_heading = vehicle_motion_heading(obj)
        aligned = (motion_heading is not None and
                   abs((motion_heading-math.radians(ego["heading_deg"])+math.pi) %
                       (2*math.pi)-math.pi) < math.radians(30))
        rear_approach = bool(native_motion and aligned and
                             longitudinal[0] < -EGO_HALF_LENGTH_M and not physical_hit[0])
        if ignore_rear_approach and rear_approach and not physical_hit.any():
            # Waive only a headway/shell veto on a body-safe trajectory.
            # Neither current nor forecast physical contact can be hidden.
            continue
        lateral_v = abs(float(np.asarray([vx, vy]) @ en[0]))
        # Offset and drift relative to our path (not our heading, which curves),
        # so a slow car easing into our lane is seen early (auto run 51
        # 2026-10-04: 0.2-0.5 m/s drift from 3.6 m was ignored, contact at 33 km/h).
        if _prepared is None:
            k = int(np.argmin(np.linalg.norm(world_path-np.asarray(obj[:2]), axis=1)))
            tangent = world_path[min(k+1, len(world_path)-1)]-world_path[max(k-1, 0)]
            tangent /= max(1e-6, float(np.linalg.norm(tangent)))
            normal = np.array([-tangent[1], tangent[0]])
            path_offset = float((np.asarray(obj[:2])-world_path[k]) @ normal)
            drift_toward = -math.copysign(1., path_offset)*float(np.asarray([vx, vy]) @ normal)
        else:
            path_offset, drift_toward = _prepared.path_relation(obj)
        if (not lane_change and not native_motion and obj[4] == "npc" and aligned and not physical_hit.any() and
                lateral[0] > .946 + width/2 + .4 and lateral_v < .5 and
                abs(path_offset) > 2.8 and drift_toward < .25):
            # A same-flow car in the next lane (beside, ahead or behind) that is
            # not drifting toward us: keep our speed and pass it. Straight-line
            # prediction on curves made it "cross" our path and slowed us.
            continue
        if (not lane_change and not native_motion and obj[4] == "npc" and aligned and
                longitudinal[0] < -EGO_HALF_LENGTH_M and not physical_hit.any()):
            # Braking for a same-flow car approaching from behind worsens its
            # closing gap. It still vetoes entry into that lane; current body
            # overlap and all front/crossing objects retain the collision check.
            continue
        if lane_change and obj[4] == "npc" and aligned:
            ego_v = np.gradient(distances, times)
            rear_closing = np.maximum(0., np.asarray([vx, vy]) @ et.T - ego_v)
            other_long, other_lat = projected_halves(length, width, angle-yaw)
            rear_gap = -longitudinal - EGO_HALF_LENGTH_M - other_long
            same_lane = lateral < EGO_HALF_WIDTH_M + other_lat + .25
            if native_motion:
                # Veto entry toward a rear car, while allowing a footprint-
                # checked escape that increases separation from its lane.
                same_lane &= lateral <= lateral[0]+.05
            # A rear car is a veto for entering its lane, not a stop command.
            # Enter ahead of it only with 1.5 s of closing time and a gap in which
            # it can match our speed at <= 3.5 m/s^2 (it never has to brake hard).
            # Zipper merge at walking pace (both nearly standing): the queued car
            # behind needs 2 m plus its own stopping distance at 2 m/s^2.
            other_speed = float(np.hypot(vx, vy))
            base_gap = (1.5+other_speed if zipper and other_speed < ZIPPER_SPEED_MPS else 5.)
            hit |= same_lane & (longitudinal < 0.) & (
                rear_gap < base_gap + np.maximum(rear_closing*1.5, rear_closing**2/7.))
        if ignore_rear_approach and not footprint_hit.any():
            # Admission-only rear/side vetoes must not mask a later crossing
            # body when checking execution of the already selected path.
            continue
        if hit.any():
            when = float(times[np.flatnonzero(hit)[0]])
            if earliest is None or when < earliest["when"]:
                earliest = dict(when=when, name=obj[3], object_id=obj[11] if len(obj)>11 else None,
                                collision_when=(float(times[np.flatnonzero(footprint_hit)[0]])
                                                if footprint_hit.any() else None),
                                physical_collision_when=(float(times[np.flatnonzero(physical_hit)[0]])
                                                         if physical_hit.any() else None),
                                footprint_margin_m=footprint_margin_m,
                                reason="predicted lane-change gap occupied")
                earliest['rear_approach'] = rear_approach
                first_physical = np.flatnonzero(physical_hit)
                earliest['rear_body_conflict'] = bool(aligned and len(first_physical) and
                    longitudinal[first_physical[0]] < -EGO_HALF_LENGTH_M)
    return earliest


# Exact identity keeps custom functions and mocked checkers on their original
# interface. Prepared state is private to the production implementation.
_PREPARED_TIMED_GAP = timed_gap


def target_lane_rear_conflict(ego, objects, offset, speed, zipper=False,
                              body_shift_m=EGO_CENTRE_M):
    """A same-flow car in the lane `offset` m to our left (ego frame) beside the
    body or right behind it: rear gap < 3 m + 1 s of closing (3 m + 0.5 s for a
    faster car, which passes us anyway). The timed footprint and rear-gap check,
    which models our acceleration, still has to pass as well. None when the lane
    may be entered now."""
    for obj in objects:
        if obj[4] != "npc":
            continue
        motion_heading = vehicle_motion_heading(obj)
        if motion_heading is not None:
            angle = (motion_heading-math.radians(ego["heading_deg"])+math.pi) % (2*math.pi)-math.pi
            if abs(angle) > math.radians(30):
                continue
        x, y = local_points(np.asarray(obj[:2], dtype=float)[None, :], ego)[0]
        support, across = projected_support(obj, math.radians(ego["heading_deg"]))
        ahead = EGO_HALF_LENGTH_M+body_shift_m+support+(.5 if zipper else 1.)
        if abs(y-offset) > 1.75+across or x > ahead or x < -90.:
            continue
        yaw = math.radians(ego["heading_deg"])
        along_speed = (float(obj[9])*math.cos(yaw)+float(obj[10])*math.sin(yaw)
                       if len(obj) > 10 else float(obj[5]))
        closing = max(0., along_speed-speed)
        rear_gap = -x-(EGO_HALF_LENGTH_M-body_shift_m)-support
        # A faster car passes on its own: veto only while it is beside or about to
        # be (then the timed footprint / rear-gap check decides between ahead of it
        # and behind it). A car at our speed never clears by itself.
        if zipper and along_speed < ZIPPER_SPEED_MPS:
            # Walking-pace zipper: we move off with the queue, so no closing term;
            # 1.5 m plus 1 s of its speed behind our rear, so it never has to brake
            # for us (MORAI queue cars did not avoid us: run 266, car id 3).
            conflict = rear_gap < 1.5+along_speed
        else:
            conflict = rear_gap < (3.+.5*closing if closing > 1. else 3.+closing)
        if conflict:
            return dict(name=obj[3], object_id=obj[11] if len(obj) > 11 else None,
                        speed_mps=round(along_speed, 2), along_m=round(float(x), 1),
                        rear_gap_m=round(float(rear_gap), 1), closing_mps=round(closing, 1),
                        ttc_s=round(float(rear_gap)/closing, 1) if closing > .1 and rear_gap > 0 else None)
    return None


def lead_vehicle_speed_cap(path, ego, objects, target_mps, body_shift_m=EGO_CENTRE_M):
    """Headway limit for measured same-flow vehicles in the selected path."""
    path = np.asarray(path, dtype=float)[:, :2]
    speed = max(0., float(ego.get("speed_kmh", 0.)))/3.6
    cap = float(target_mps)
    for obj in objects:
        if obj[4] != "npc":
            continue
        position = local_points(np.asarray(obj[:2])[None, :], ego)[0]
        nearest = int(np.argmin(np.linalg.norm(path-position, axis=1)))
        tangent = path[min(nearest+1, len(path)-1)]-path[max(0, nearest-1)]
        tangent /= max(1e-6, np.linalg.norm(tangent))
        lateral_gap = abs(float((position-path[nearest]) @ [-tangent[1], tangent[0]]))
        motion_heading = vehicle_motion_heading(obj)
        if motion_heading is None:
            continue
        angle = (motion_heading-math.radians(ego["heading_deg"])+math.pi) % (2*math.pi)-math.pi
        support, across = projected_support(obj, math.radians(ego["heading_deg"]) +
                                            math.atan2(tangent[1], tangent[0]))
        if position[0] <= 0 or abs(angle) > math.radians(30) or lateral_gap > EGO_HALF_WIDTH_M+across+.3:
            continue
        gap = max(0., position[0]-(EGO_HALF_LENGTH_M+body_shift_m)-support)
        cap = min(cap, max(0., obj[5]+GAP_GAIN*(gap-max(MIN_GAP_M, speed*HEADWAY_S))))
    return cap


class NearFieldMemory:
    """Keep repeated stationary returns while they enter the VLP16 near-field blind zone."""
    # A blind cell is re-confirmed only while the car moves past it. Without a
    # bound, a parked car keeps its last returns forever and the path never
    # clears (2026-10-03 highway: 3 m "blocked" with a free live scan).
    MAX_BLIND_AGE_S = 1.5

    def __init__(self, retain_measured_points=False, hold_blind=False):
        self.cells = {}
        self.retain_measured_points = bool(retain_measured_points)
        # hold_blind (tunnel): a stationary return stays while it is still inside the
        # blind zone ahead, however long the car stands. A 0.8 m concrete barrier ~2 m
        # ahead of the bumper is below the lowest beam; the 1.5 s bound forgot it and
        # the car drove into it at full throttle (auto run 196). Movers, leaving the
        # zone (passing it, reversing away) and a 10 m pose jump still clear it.
        self.hold_blind = bool(hold_blind)
        self.last_scan = None
        self.last_ego = None
        self.retained_n = 0

    def update(self, points, ego, scan_stamp, moving_objects=()):
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if self.last_ego is not None and np.linalg.norm(np.array([ego["x"], ego["y"]])-self.last_ego) > 10.:
            self.cells.clear()
        self.last_ego = np.array([ego["x"], ego["y"]])
        if scan_stamp != self.last_scan:
            self.last_scan = scan_stamp
            near = points[(points[:, 0] > 2.3675) & (points[:, 0] < 9.) & (np.abs(points[:, 1]) < 2.5)]
            world = world_points(near, ego)
            if len(world) >= 4:
                keys, indices = np.unique(np.round(world/.4).astype(int), axis=0, return_index=True)
                for key, index in zip(keys, indices):
                    key = tuple(key)
                    old = self.cells.get(key)
                    count = old[1]+1 if old and 0 <= scan_stamp-old[0] <= .4 else 1
                    self.cells[key] = ((scan_stamp, count, world[index].copy())
                                       if self.retain_measured_points else (scan_stamp, count))
        retained = []
        for key, observation in list(self.cells.items()):
            stamp, count = observation[:2]
            world = observation[2] if self.retain_measured_points else np.array(key)*.4
            local = local_points(world[None, :], ego)[0]
            age = scan_stamp-stamp
            occupied_by_mover = False
            for obj in moving_objects:
                minimum_speed = .15 if obj[4] == "pedestrian" else .5
                if obj[4] not in ("npc", "pedestrian") or obj[5] < minimum_speed:
                    continue
                yaw = math.radians(obj[8])
                position = np.asarray(obj[:2], dtype=float)
                positions = [position]
                if len(obj) > 10 and (len(obj) <= 14 or obj[14]):
                    # Velocity becomes available after several observations.
                    # Clear returns from those earlier footprints as well.
                    positions.append(position-np.asarray(obj[9:11])*np.clip(age, 0., .8))
                for centre in positions:
                    delta = world-centre
                    along = delta @ [math.cos(yaw), math.sin(yaw)]
                    across = delta @ [-math.sin(yaw), math.cos(yaw)]
                    if abs(along) < obj[6]/2+.8 and abs(across) < obj[7]/2+.8:
                        occupied_by_mover = True
                        break
                if occupied_by_mover:
                    break
            blind = -2.5 < local[0] < 6.5 and abs(local[1]) < 3.
            if (age < 0 or occupied_by_mover or local[0] < -2.5 or np.linalg.norm(local) > 12. or
                    (not blind and age > .5) or (age > self.MAX_BLIND_AGE_S and not self.hold_blind)):
                del self.cells[key]
            elif count >= 2 and blind and age > .08:
                retained.append(local)
        self.retained_n = len(retained)
        return np.vstack((points, retained)) if retained else points


STATIC_STANDOFF_M = 12.
EXIT_SPEED_MPS = 50./3.6
EXIT_DECEL_MPS2 = 2.5
EXIT_MARGIN_M = 40.
# Fractions of the current cap tried, highest first, when a footprint conflict is
# predicted on the path we drive.
PACE_FACTORS = (.95, .9, .85, .8, .7, .6, .5, .4, .3, .2)
RETURN_SEARCH_MPS = 50./3.6
# Off the lane by this much without a lane change: go back as a checked change.
RECOVER_OFFSET_M = 1.2
# Below this (10 km/h) a merge out of a closed lane may use queue-sized gaps.
ZIPPER_SPEED_MPS = 10./3.6
# Assumed sideways speed of a car merging out of a closing lane (run 243: 0.2-0.35).
MERGE_LATERAL_MPS = .4
YIELD_MAX_DECEL_MPS2 = 7.
HIGHWAY_MAP_FLOOR_MPS = 50./3.6   # lane rule, not an obstacle: 30 held the highway entry at 25-33 km/h (auto runs 109-114; user: never < 50)


class HighwayPlanner:
    """At least two lane changes, with further checked overtakes when traffic blocks cruising."""
    def __init__(self, config, route, body_shift_m=EGO_CENTRE_M):
        self.body_shift_m = float(body_shift_m)
        self.front_m = EGO_HALF_LENGTH_M+self.body_shift_m
        self.rear_m = EGO_HALF_LENGTH_M-self.body_shift_m
        self.config = json.loads(Path(config).read_text()) if isinstance(config, (str, Path)) else config
        self.start = int(self.config["from_idx"])
        self.end = int(self.config["to_idx"])
        # New (out / further-overtake) lane changes must end before this index:
        # the exit work-zone taper (drum lines from idx ~3045, every lane merging at
        # idx ~3200) is no place to start one. A pass into the closing right lane
        # stopped the car behind the drums for good (sim, 2026-10-05).
        self.change_until = int(self.config.get("lane_change_until_idx", self.end - 460))
        # Route index ranges where the neighbouring lane on `side` (+1 left) is
        # closed into ours: the exit work-zone drum taper (idx 3030-3210). A car in
        # that lane may move over anywhere in the zone (auto run 243: a 29 km/h car
        # merged in front of us at 78 km/h). The on-ramp lane end (idx 2430-2610)
        # is not one: there our lane ends and the gap check paces our merge.
        default_zones = ([[3030, 3210, 1]]
                         if self.config.get("start_link") == "A2256W000411" else [])
        self.merge_zones = [tuple(int(v) for v in zone)
                            for zone in self.config.get("merge_zones", default_zones)]
        self.target_mps = float(self.config["kmh"])/3.6
        self.overtake_mps = float(self.config.get("overtake_kmh", self.config["kmh"]))/3.6
        if not math.isfinite(self.overtake_mps) or self.overtake_mps < self.target_mps:
            raise ValueError("Highway overtaking limit must be finite and at least the cruise speed")
        self.route = np.asarray(route, dtype=float)
        self.station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.route, axis=0), axis=1))]
        self.phase, self.completed, self.target_offset = "OUTSIDE", 0, 0.
        self.saved_world = None
        self.hold_station = None
        self.selected_side = 0
        self.lane_side = 0
        self.lane_profile = None
        self.status = {}
        self.free_three_lane = self.config.get("lane_change_policy") == "free_three_lane"
        self.longitudinal = self.new_speed_planner()
        self.three_lane_policy = None
        self._speed_hold_goal = None
        self._speed_hold_until = 0.
        self._speed_hold_stamp = None
        self._speed_hold_recovery = None

    def active(self, index):
        return self.start <= index <= self.end

    def new_speed_planner(self):
        if self.free_three_lane:
            return HighwaySpeedPlanner(self.config.get("comfort_decel_mps2", 1.2),
                                       self.config.get("comfort_jerk_mps3", 2.),
                                       self.config.get("emergency_jerk_mps3", 12.),
                                       body_shift_m=self.body_shift_m)
        return HighwaySpeedPlanner(body_shift_m=self.body_shift_m)

    def reset(self):
        self.phase, self.completed, self.target_offset = "OUTSIDE", 0, 0.
        self.saved_world, self.hold_station, self.selected_side = None, None, 0
        self.lane_side, self.lane_profile = 0, None
        self.longitudinal = self.new_speed_planner()
        self.three_lane_policy = None
        self._speed_hold_goal = None
        self._speed_hold_until = 0.
        self._speed_hold_stamp = None
        self._speed_hold_recovery = None

    def predicted_gap(self, path, ego, objects, target_mps, horizon_s=7.,
                      lane_change=True, zipper=False):
        trajectory = self.longitudinal.preview(path, ego, objects, target_mps, horizon_s,
                                               ceiling_mps=self.target_mps)
        return timed_gap(path, ego, objects, target_mps, horizon_s, lane_change, zipper,
                         speed_trajectory=trajectory,
                         object_positions=self.longitudinal.object_positions,
                         object_yaws=self.longitudinal.object_yaws, body_shift_m=self.body_shift_m)

    def plan_speed(self, path, ego, objects, goal_mps, static_clear_m=170.,
                   static_mover_mps=0., emergency=False, ceiling_mps=None,
                   validation_stamp=None):
        """Check and publish one gradual, headway-aware longitudinal plan.

        Called after the GUI's final nudge and side-gap limits. The exact
        trajectory behind the command must still pass the footprint forecast
        and the stationary-obstacle stopping envelope.
        """
        try:
            normalize_path(path)
        except (ValueError, TypeError):
            emergency = True
        if emergency:
            self._speed_hold_goal = None
            self._speed_hold_until = 0.
            self._speed_hold_recovery = None
            return dict(reference_mps=0., emergency=True,
                        diagnostics=dict(reference_checked=False, emergency=True))
        stamp = float(ego.get("stamp", time.monotonic()))
        ceiling = self.target_mps if ceiling_mps is None else min(self.overtake_mps, max(0., ceiling_mps))
        requested = max(0., float(goal_mps))
        if self._speed_hold_stamp is not None and stamp < self._speed_hold_stamp:
            self._speed_hold_goal = None
            self._speed_hold_until = 0.
            self._speed_hold_recovery = None
        self._speed_hold_stamp = stamp
        if self._speed_hold_goal is None or requested < self._speed_hold_goal - .2:
            self._speed_hold_goal, self._speed_hold_until = requested, stamp + .6
            self._speed_hold_recovery = None
        goal = min(requested, self._speed_hold_goal) if stamp < self._speed_hold_until else requested
        if stamp >= self._speed_hold_until:
            self._speed_hold_goal = requested
            self._speed_hold_recovery = None
        initial_goal = goal
        changing = self.saved_world is not None
        execution_margin = (float(self.status.get('command_footprint_margin_m',
                            LANE_ADMISSION_MARGIN_M)) if changing else CURRENT_PATH_FOOTPRINT_MARGIN_M)
        command_objects = objects
        from planning.highway.policy import _cached_gap_checker, _paced_previews
        gap_check = _cached_gap_checker(path, ego, self.longitudinal.object_positions,
                                       self.longitudinal.object_yaws, self.body_shift_m,
                                       gap_function=timed_gap)
        command_check = _cached_command_preview(self.longitudinal)
        if changing and self.free_three_lane:
            from planning.highway.policy import lane_centred_copies
            command_objects = list(objects)+lane_centred_copies(
                self.longitudinal.motion_paths, objects, path, ego, horizon_s=18.)
        intrusion_reserves = []
        if self.free_three_lane:
            world_path = world_points(np.asarray(path)[:, :2], ego)
            ego_speed = max(0., float(ego.get('speed_kmh', 0.)))/3.6
            for obj in objects:
                if obj[4] != 'npc' or len(obj)<11 or len(obj)>14 and not obj[14]:
                    continue
                at = int(np.argmin(np.linalg.norm(world_path-np.asarray(obj[:2]), axis=1)))
                tangent = world_path[min(at+1, len(world_path)-1)]-world_path[max(0, at-1)]
                tangent /= max(1e-6, np.linalg.norm(tangent))
                normal = np.array([-tangent[1], tangent[0]])
                offset = float((np.asarray(obj[:2])-world_path[at]) @ normal)
                toward = -math.copysign(1., offset)*float(np.asarray(obj[9:11]) @ normal)
                native_motion = self.longitudinal.motion_paths.get(obj[11] if len(obj)>11 else id(obj))
                if native_motion is not None:
                    # Ego steering rotates the selected path relative to a
                    # parallel car. Only measured motion across that car's
                    # surveyed lane is a cut-in; timed hull checks still cover
                    # our own lane change into its lane.
                    native_path = native_motion[0]
                    native_at = int(np.argmin(np.linalg.norm(native_path-np.asarray(obj[:2]),axis=1)))
                    native_tangent = native_path[min(native_at+1,len(native_path)-1)] - \
                                     native_path[max(0,native_at-1)]
                    native_tangent /= max(1e-6,float(np.linalg.norm(native_tangent)))
                    native_normal = np.array([-native_tangent[1],native_tangent[0]])
                    toward = -math.copysign(1., offset)*float(np.asarray(obj[9:11]) @ native_normal)
                closing = ego_speed-float(np.asarray(obj[9:11]) @ tangent)
                along = float((np.asarray(obj[:2])-[ego['x'], ego['y']]) @ tangent)
                support, across = projected_support(obj, math.atan2(tangent[1], tangent[0]))
                lateral_gap = max(0., abs(offset)-EGO_HALF_WIDTH_M-across-.3)
                if (0. < along < 120. and toward >= .25 and closing > .5 and
                        abs(offset) < 5.5 and lateral_gap/toward < 3.):
                    gap = along-self.front_m-support-.6
                    intrusion_reserves.append((gap, closing))
        preview_options = dict(static_clear_m=static_clear_m, static_mover_mps=static_mover_mps,
                               standoff_m=STATIC_STANDOFF_M, ceiling_mps=ceiling)
        def forecast(pace, decel=None, trajectory=None):
            if trajectory is None:
                trajectory = self.longitudinal.preview(path, ego, objects, pace,
                    decel_mps2=decel, **preview_options)
            execution_risk, execution_unsafe = None, False
            def command_risk(candidate, braking_limit=None):
                executed = command_check(
                    candidate.prefix(TRAFFIC_CONTROL_HORIZON_S))
                if (braking_limit is not None and
                        (not np.isfinite(executed.accelerations).all() or
                         np.any(executed.accelerations < -braking_limit-1e-9))):
                    return None, True
                collision = gap_check(command_objects, candidate.reference_mps, executed,
                    lane_change=changing, ignore_rear_approach=not changing,
                    footprint_margin_m=execution_margin)
                blocked = collision is not None and (changing or
                    collision.get('collision_when') is not None or
                    collision.get('reason') == 'lane-change path too short' or
                    collision.get('reason', '').startswith('invalid highway'))
                stop_line = static_clear_m+static_mover_mps*executed.times
                return collision, blocked or bool(np.any(executed.distances>stop_line-.25))
            execution_risk, execution_unsafe = command_risk(trajectory)
            trajectory.execution_reference_checked = True
            if (execution_unsafe and trajectory.reference_mps > trajectory.speeds[0]+.01
                    and trajectory.reference_time_s > CONTROL_LOOKAHEAD_S):
                # A future positive reference can release today's brake
                # before a crossing body clears. Retain the short reference
                # only when executing it is safe too; clear-road recovery
                # keeps its checked acceleration without another stop hold.
                immediate = float(np.interp(CONTROL_LOOKAHEAD_S,
                    trajectory.times, trajectory.speeds))
                trajectory.recovery_brake_retained = bool(immediate<trajectory.speeds[0]-.01)
                trajectory.acceleration_reference_limited = True
                trajectory.reference_override_s = CONTROL_LOOKAHEAD_S
                execution_risk, execution_unsafe = command_risk(trajectory)
            risk = gap_check(objects, pace, trajectory.prefix(TRAFFIC_CONTROL_HORIZON_S),
                lane_change=changing,
                ignore_rear_approach=True,
                footprint_margin_m=CURRENT_PATH_FOOTPRINT_MARGIN_M)
            # A rear headway veto is not a command to brake in front of it.
            nominal_unsafe = bool(risk is not None and (
                risk.get("collision_when") is not None or
                risk.get("reason") == "lane-change path too short" or
                risk.get("reason", "").startswith("invalid highway")))
            unsafe = execution_unsafe or nominal_unsafe
            if execution_unsafe and execution_risk is not None:
                risk = execution_risk
            stop_line = static_clear_m + static_mover_mps * trajectory.times
            static_unsafe = bool(np.any(trajectory.distances > stop_line - .25))
            # A measured cut-in must retain relative stopping room plus the
            # follower response delay. A trajectory that merely misses the
            # current lateral extrapolation cannot waive this reserve.
            intrusion_unsafe = any(gap < closing*.35 + closing*closing/
                (2.*trajectory.braking_limit_mps2) for gap, closing in intrusion_reserves)
            unsafe = unsafe or static_unsafe or intrusion_unsafe
            if (decel in (4., 5.) and execution_unsafe and not nominal_unsafe and
                    not static_unsafe and not intrusion_unsafe and
                    trajectory.reference_mps < trajectory.speeds[0]-.01):
                # A short brake reference can settle above the safe nominal
                # trajectory. Try lower references on that same checked ramp,
                # highest first; safety need not be monotone in reference time.
                end = min(TRAFFIC_CONTROL_HORIZON_S, float(trajectory.times[-1]))
                candidates = [replace(trajectory, reference_override_s=float(at))
                    for at in np.arange(CONTROL_LOOKAHEAD_S+.05, end+.025, .05)]
                candidates.sort(key=lambda candidate: candidate.reference_mps, reverse=True)
                seen = set()
                trials = 0
                for candidate in candidates:
                    reference = candidate.reference_mps
                    if reference >= trajectory.reference_mps-1e-9 or reference in seen:
                        continue
                    seen.add(reference)
                    trials += 1
                    candidate.braking_reference_extended = True
                    candidate.braking_reference_trials = trials
                    candidate_risk, blocked = command_risk(candidate,
                        braking_limit=trajectory.braking_limit_mps2)
                    if not blocked:
                        trajectory, risk, unsafe = candidate, candidate_risk, False
                        break
            return trajectory, risk, unsafe
        hold_released = False
        if goal < requested - .2:
            # The caller supplies the actual validated sensor source time,
            # never a decision/receipt time. One clear frame or reuse of the
            # same scan cannot release a recently lowered safety goal.
            fresh = (validation_stamp is not None and math.isfinite(validation_stamp) and
                     validation_stamp > 0. and -.05 <= stamp-validation_stamp <= .35)
            state = self._speed_hold_recovery
            if not fresh:
                self._speed_hold_recovery = None
            elif state is None or validation_stamp != state[0]:
                _, _, recovery_unsafe = forecast(requested)
                if recovery_unsafe:
                    self._speed_hold_recovery = None
                else:
                    continuous = state is not None and 0. < validation_stamp-state[0] <= .35
                    since = state[1] if continuous else validation_stamp
                    count = state[2]+1 if continuous else 1
                    self._speed_hold_recovery = validation_stamp, since, count
                    if count >= 2 and validation_stamp-since >= .15-1e-9:
                        goal = initial_goal = requested
                        self._speed_hold_goal, self._speed_hold_until = requested, stamp
                        self._speed_hold_recovery = None
                        hold_released = True
        else:
            self._speed_hold_recovery = None
        trajectory, risk, unsafe = forecast(goal)
        rear_escape = False
        if (unsafe and risk is not None and risk.get('rear_body_conflict') and
                goal < requested-.2):
            # A temporary slow hold can invite actual rear contact. Before
            # braking harder, try the already requested pace within all current
            # caps. Release the hold only on body-safe nominal and actuator
            # forecasts, including the front/static and changing-lane checks.
            escaped, escape_risk, escape_unsafe = forecast(requested)
            if not escape_unsafe:
                trajectory, risk, unsafe, goal = escaped, escape_risk, False, requested
                self._speed_hold_goal, self._speed_hold_until = requested, stamp
                self._speed_hold_recovery = None
                rear_escape = True
        if unsafe:
            # Search a locally continuous pace boundary instead of toggling
            # between coarse fractions such as 90 and 45 km/h.
            upper = goal
            paces = np.r_[np.arange(max(0., goal-.5), 0., -.5), 0.]
            for pace, prepared in _paced_previews(self.longitudinal, path, ego, objects,
                                                  paces, 7., **preview_options):
                candidate, candidate_risk, candidate_unsafe = forecast(float(pace), trajectory=prepared)
                if not candidate_unsafe:
                    lower = float(pace)
                    for _ in range(3):
                        middle = (lower + upper) / 2.
                        refined, refined_risk, refined_unsafe = forecast(middle)
                        if refined_unsafe:
                            upper = middle
                        else:
                            lower, candidate, candidate_risk = middle, refined, refined_risk
                    trajectory, risk, unsafe, goal = candidate, candidate_risk, False, lower
                    break
                upper = float(pace)
        if unsafe:
            # Escalate checked braking before sending a zero-speed emergency.
            # Keep the new policy's strong forecast within a conservative
            # 5 m/s². An infeasible plan triggers immediate emergency control
            # instead of approving a delayed soft reference.
            braking_levels = (1.5, 2., 2.5, 4., 5.) if self.free_three_lane else (4., 6.)
            # The pace search above already includes a stop at the comfort
            # deceleration; escalate only to firmer braking than that.
            braking_levels = tuple(level for level in braking_levels
                                   if level > self.longitudinal.comfort_decel_mps2+1e-9) or (5.,)
            for decel in braking_levels:
                trajectory, risk, unsafe = forecast(0., decel=decel)
                goal = 0.
                if not unsafe:
                    break
        if unsafe:
            return dict(reference_mps=0., emergency=True,
                        diagnostics=dict(reference_checked=False, emergency=True, risk=risk,
                                         footprint_margin_m=CURRENT_PATH_FOOTPRINT_MARGIN_M,
                                         command_footprint_margin_m=execution_margin,
                                         traffic_gap_queries=dict(gap_check.stats),
                                         command_preview_queries=dict(command_check.stats),
                                         reason="no checked braking trajectory",
                                         unavoidable_rear_conflict=bool(risk and risk.get('rear_body_conflict'))))
        if goal < initial_goal - .2:
            self._speed_hold_goal, self._speed_hold_until = goal, stamp + .6
            self._speed_hold_recovery = None
        diagnostics = trajectory.diagnostics()
        diagnostics.update(requested_goal_kmh=round(requested*3.6, 2),
                           held=goal < requested-.2, emergency=False,
                           hold_released=hold_released, rear_escape=rear_escape,
                           footprint_margin_m=CURRENT_PATH_FOOTPRINT_MARGIN_M,
                           traffic_check_horizon_s=TRAFFIC_CONTROL_HORIZON_S,
                           command_footprint_margin_m=execution_margin,
                           traffic_gap_queries=dict(gap_check.stats),
                           command_preview_queries=dict(command_check.stats),
                           headway_s=HEADWAY_S, minimum_gap_m=MIN_GAP_M,
                           time_scale=round(clip_rtf(self.longitudinal.time_scale), 3))
        return dict(reference_mps=trajectory.reference_mps, emergency=False,
                    diagnostics=diagnostics)

    def unblock_change(self, route, path, ego, speed, map_clearance, static_clearance):
        """Re-plan a committed lane change whose path has run into static obstacles.

        A return accepted at idx 2957 with the 112 m change crossed the lanes right
        where the exit taper's drum line separates them; the stopping reserve held
        the car 26 m short of it and, the saved path never changing, it stood there
        for 72 s (regression highway_random_3, 2026-10-05). Tries a shorter crossing
        to the same lane, then going back to the lane it left (the change is retried
        later through the normal checks); keeps the path unless one is clear for
        clearly longer."""
        clear = float(static_clearance(path, 170.))
        clear = clear if math.isfinite(clear) else 0.
        # Only once the obstacle is close (the reserve cap is already braking for
        # it); a far one is left to the stopping-reserve cap.
        if clear >= STATIC_STANDOFF_M+8.+speed:
            return path, None
        origin = getattr(self, "change_origin", None)
        options = [(self.lane_side, self.target_offset, False)]
        if origin is not None:
            options.append((origin[0], origin[1], True))
        best = None
        for side, offset, back in options:
            for length in dict.fromkeys((max(15., speed*3.), 30., 60.)):
                candidate = lane_change_path(route, self.offsets_along(route, side, offset), length)
                candidate_clear = float(static_clearance(candidate, 170.))
                corridor = float(map_clearance(candidate, length+12.))
                if (not math.isfinite(candidate_clear) or candidate_clear < clear+10. or
                        not math.isfinite(corridor) or corridor < min(length+10., 30.)):
                    continue
                score = candidate_clear - (15. if back else 0.)
                if best is None or score > best[0]:
                    best = (score, candidate, side, offset, back)
        if best is None:
            return path, None
        _, candidate, side, offset, back = best
        if back:
            self.lane_side, self.target_offset, self.phase = origin
            self.saved_world = None
            return candidate, "committed lane change blocked by static obstacles: back to the lane it left"
        self.saved_world = world_points(candidate, ego)
        return candidate, "committed lane change blocked by static obstacles: shorter crossing"

    def merge_zone_ahead(self, index, side, station_ahead):
        """Distance (m) from the ego to the start of a merge zone on `side` that has
        not ended before `station_ahead` m; None when there is none."""
        here = float(self.station[index])
        for start, stop, zone_side in self.merge_zones:
            if zone_side != side:
                continue
            s0 = float(self.station[min(start, len(self.station)-1)])-here
            s1 = float(self.station[min(stop, len(self.station)-1)])-here
            if s1 > 0. and s0 < station_ahead:
                return max(0., s0)
        return None

    def side_traffic_speed_cap(self, path, ego, objects, speed, index):
        """Yield only to side traffic that can enter the selected highway path.

        * The car may come into our lane (its lane closes in a merge zone, or it
          drifts toward us / sits close to our lane): pass only when we are clear
          ahead of it (10 m) before it can reach our lane, otherwise fall in behind
          it with the lead-vehicle headway (auto run 243: we drove past a merging
          29 km/h car at 78 km/h and needed a 75 -> 13 km/h emergency stop).
        Cars we are already level with are left to the collision check: braking
        beside them only keeps us there. Parallel cars in a separate lane do
        not impose a speed-difference limit on a clear overtaking path."""
        cap, reason, info = float("inf"), None, None
        yield_cap, yield_reason, yield_info = float("inf"), None, None
        level_merger = None
        path = np.asarray(path, dtype=float)[:, :2]
        yaw = math.radians(ego["heading_deg"])
        heading = np.array([math.cos(yaw), math.sin(yaw)])
        for obj in objects:
            if obj[4] != "npc" or len(obj) < 11:
                continue
            motion_heading = vehicle_motion_heading(obj)
            if motion_heading is None:
                continue
            angle = (motion_heading-yaw+math.pi) % (2*math.pi)-math.pi
            if abs(angle) > math.radians(30):
                continue
            position = local_points(np.asarray(obj[:2], dtype=float)[None, :], ego)[0]
            length, width = max(.3, float(obj[6])), max(.3, float(obj[7]))
            support, across = projected_support(obj, yaw)
            front_reach, rear_reach = self.front_m+support, self.rear_m+support
            if position[0] < -rear_reach-3. or position[0] > 150.:
                continue
            k = int(np.argmin(np.linalg.norm(path-position, axis=1)))
            tangent = path[min(k+1, len(path)-1)]-path[max(k-1, 0)]
            tangent /= max(1e-6, float(np.linalg.norm(tangent)))
            normal = np.array([-tangent[1], tangent[0]])
            offset = float((position-path[k]) @ normal)
            side = 1 if offset > 0. else -1
            velocity = np.array([float(obj[9]), float(obj[10])])
            other = float(velocity @ heading)
            # Lateral motion toward our path (path frame, rotated to world by ego yaw).
            world_normal = np.array([math.cos(yaw)*normal[0]-math.sin(yaw)*normal[1],
                                     math.sin(yaw)*normal[0]+math.cos(yaw)*normal[1]])
            drift = -side*float(velocity @ world_normal)
            zone = self.merge_zone_ahead(index, side, max(0., position[0])+150.)
            if (position[0] < 1. and 1. < abs(offset) < 3.6 and
                    (zone is not None and zone <= 0. or drift > .2 or abs(offset) < 2.9)):
                # Beside or just behind us and coming over: braking now would put
                # it into our side (sim highway_random_1: a yield for the car ahead slowed
                # us beside a 40 km/h car merging from just behind, 0.15 m).
                level_merger = obj[11] if len(obj) > 11 else True
            if position[0] < -rear_reach or not 1.9+across <= abs(offset) <= 5.5:
                continue        # behind us, in our path (lead headway) or beyond the next lane
            gap = position[0]-front_reach            # our front to its rear
            edge = EGO_HALF_WIDTH_M+across+.3             # lateral offset at which it touches our lane
            if other >= speed-.5:
                continue        # not slower than us: nothing to pass
            reasons = []
            if zone is not None:
                reasons.append("its lane merges into ours")
            if drift > .2 and abs(offset) < 3.6:
                reasons.append("drifting toward us")
            if abs(offset) < 2.9:
                reasons.append("close to our lane")
            limit, why = None, None
            if reasons and position[0] > 0.:
                closing = max(.1, speed-other)
                clear_ahead = (position[0]+rear_reach+10.)/closing
                lateral_time = max(0., abs(offset)-edge)/max(drift, MERGE_LATERAL_MPS)
                if zone is not None:
                    # It may start over as soon as it reaches the zone.
                    may_enter = max(0., zone-position[0])/max(other, .5)+lateral_time
                else:
                    may_enter = lateral_time
                # Yield while we can still drop in behind it (<= 7 m/s^2 needed;
                # run 243 first saw the car 40 m ahead at 82 km/h, 7.6 m/s^2 at
                # the very end); later, braking would only keep us level with it.
                need = (speed*speed-other*other)/(2.*max(1., gap-5.))
                if clear_ahead > may_enter-1. and need <= YIELD_MAX_DECEL_MPS2:
                    limit = max(0., other+GAP_GAIN*(gap-max(MIN_GAP_M, speed*HEADWAY_S)))
                    why = "yield to merging side car: " + ", ".join(reasons)
            details = dict(object_id=obj[11] if len(obj) > 11 else None, along_m=round(float(position[0]), 1),
                           offset_m=round(offset, 2), speed_kmh=round(other*3.6, 1),
                           drift_mps=round(drift, 2))
            if limit is not None and limit < yield_cap:
                yield_cap, yield_reason = limit, "highway " + why
                yield_info = dict(details, cap_kmh=round(limit*3.6, 1), why=why)
        if yield_cap < cap and level_merger is None:
            cap, reason, info = yield_cap, yield_reason, yield_info
        if info is not None and level_merger is not None:
            info = dict(info, yield_suppressed_by=level_merger)
        return cap, reason, info

    def static_mover_speed(self, index, ego, static_available):
        """Speed (m/s) of the first static-scan obstacle when its route station
        advances steadily (>= 4 m/s for 1.2 s, < 0.8 m fit residual); else 0.

        Only before the lane-change limit: in the exit taper a drum line seen
        through a shifting nudge must never read as moving."""
        history = self.__dict__.setdefault("_static_track", [])
        stamp = float(ego.get("stamp", time.monotonic()))
        if index >= self.change_until or not 10. <= static_available < 150.:
            history.clear()
            return 0.
        station = float(self.station[index]) + static_available
        if history and (not 0. < stamp-history[-1][0] <= .5 or abs(station-history[-1][1]) > 6.):
            history.clear()
        history.append((stamp, station))
        while history[-1][0]-history[0][0] > 2.:
            history.pop(0)
        if len(history) < 12 or history[-1][0]-history[0][0] < 1.2:
            return 0.
        t, s = np.asarray(history).T
        slope, intercept = np.polyfit(t-t[0], s, 1)
        # The 4 m/s motion gate is measured (wall-stamped) evidence; the speed
        # the stopping reserve forecasts with is simulator speed.
        if slope < 4. or float(np.std(s-(intercept+slope*(t-t[0])))) > .8:
            return 0.
        simulated = slope/clip_rtf(getattr(self.longitudinal, 'time_scale', 1.))
        return float(min(simulated-1., self.target_mps))

    def offsets_along(self, route, side, fallback):
        """Signed lane-centre offset at every route point for one neighbour.

        A constant offset left the neighbour lane where its spacing from the
        route changes (4.6 m -> 3.0 m on the highway), and the map corridor
        then stopped the car inside a free lane.
        """
        route = np.asarray(route, dtype=float)[:, :2]
        if side == 0 or not self.lane_profile:
            return np.full(len(route), float(fallback))
        samples = [(station, side*offsets[side]) for station, offsets in self.lane_profile
                   if side in offsets]
        if not samples:
            return np.full(len(route), float(fallback))
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        stations, values = np.asarray(samples, dtype=float).T
        return np.interp(arc, stations, values)

    def decide(self, index, route, ego, objects, adjacent_offsets, map_clearance,
               static_clearance, perception_ready=True, rear_unclassified=None,
               lane_profile=None, lane_options=None, motion_options=None,
               path_transform=None, path_speed_cap=None):
        if not self.active(index):
            self.reset()
            return None
        if self.free_three_lane:
            from highway_three_lane import ThreeLanePolicy
            if self.three_lane_policy is None:
                self.three_lane_policy = ThreeLanePolicy(self.config)
            return self.three_lane_policy.decide(self, index, route, ego, objects,
                lane_options or [], map_clearance, static_clearance,
                perception_ready, rear_unclassified, motion_options,
                path_transform, path_speed_cap)
        route = np.asarray(route, dtype=float)[:, :2]
        self.longitudinal.begin(ego)
        self.lane_profile = lane_profile
        if self.lane_side:
            self.target_offset = float(self.offsets_along(route[:2], self.lane_side, self.target_offset)[0])
        if self.phase == "OUTSIDE":
            self.phase = "WAIT_GAP"
        speed = max(0., ego.get("speed_kmh", 0.))/3.6
        remaining = self.station[self.end]-self.station[index]
        # 4.5 seconds gives a gentle lane change even at 90 km/h.
        transition = max(60., max(speed, self.target_mps)*4.5)
        normal = np.gradient(route, axis=0)[0]
        normal /= max(1e-6, np.linalg.norm(normal))
        lateral = -float(route[0] @ np.array([-normal[1], normal[0]]))
        risk, reason = None, "highway cruise"
        target = self.target_mps
        candidate_checks = []
        gap_matching = False
        # Re-centring within the current lane is not a lane change: joining a
        # 0.7 m drift over the 60-112 m lane-change length kept the car beside
        # the lane-edge barrels until a static stop, then parked it there for
        # minutes (live 2026-10-03 21:35, idx 3066).
        if self.saved_world is None:
            self.recovering = False
        goal_offset = float(self.offsets_along(route[:2], self.lane_side, self.target_offset)[0])
        recentre = self.saved_world is None and abs(lateral - goal_offset) < RECOVER_OFFSET_M
        join = max(12., speed * 2.5) if recentre else transition
        path = lane_change_path(route, self.offsets_along(route, self.lane_side, self.target_offset), join)
        if self.saved_world is not None:
            local = local_points(self.saved_world, ego)
            nearest = int(np.argmin(np.linalg.norm(local, axis=1)))
            path = local[nearest:]
            if len(path) < 8:
                self.saved_world = None
                path = lane_change_path(route, self.offsets_along(route, self.lane_side, self.target_offset),
                                        transition)
            elif abs(lateral-self.target_offset) < .35:
                tangent = path[min(5, len(path)-1)]-path[0]
                if abs(math.atan2(tangent[1], tangent[0])) < math.radians(8) and getattr(self, "recovering", False):
                    # Back on our own lane after a stray offset: not a lane change.
                    self.recovering = False
                    self.saved_world = None
                elif abs(math.atan2(tangent[1], tangent[0])) < math.radians(8):
                    self.completed += 1
                    self.saved_world = None
                    self.phase = "HOLD" if abs(self.target_offset) > .5 else "DONE"
                    self.hold_station = self.station[index]
        current_map = float(map_clearance(path, 170.))
        current_static = float(static_clearance(path, 170.))
        if not math.isfinite(current_map):
            current_map = 0.
        if not math.isfinite(current_static):
            current_static = 0.
        current_lead_cap = lead_vehicle_speed_cap(path, ego, objects, self.target_mps, self.body_shift_m)
        following = self.longitudinal.preview(path, ego, objects, self.target_mps)
        unrestricted = self.longitudinal.preview(path, ego, (), self.target_mps)
        following_limited = following.speeds[-1] < unrestricted.speeds[-1] - .5
        current_gap = self.predicted_gap(path, ego, objects, self.target_mps,
                                lane_change=self.saved_world is not None)
        cruise_reserve = self.target_mps*.4 + self.target_mps**2/8. + 5.
        change_room = (self.station[min(self.change_until, len(self.station)-1)] - self.station[index]
                       if self.change_until > index else 0.)
        wants_pass = (self.phase == "DONE" and (change_room > 70. or abs(lateral) > 1.5) and
                      (min(current_map, current_static) < cruise_reserve or
                       current_lead_cap < self.target_mps-.5 or
                       following_limited or
                       current_gap is not None and current_gap.get("collision_when") is not None))
        wants_return = (self.phase == "HOLD" and
                        self.station[index]-(self.hold_station or self.station[index]) >= 25.)
        # Off our lane by a lane-change-sized amount without a lane change (run 266:
        # the car lagged the route's left shift in the on-ramp merge, sat 1.9 m right
        # of it, and every frame re-planned a 112 m join from where it was, so it
        # never came back and drove on the lane line into the exit-taper drums,
        # where it stood 166 s): go back as a checked, committed lane change.
        wants_recover = (self.saved_world is None and not wants_return and not wants_pass and
                         self.phase in ("WAIT_GAP", "HOLD", "DONE") and
                         abs(lateral - goal_offset) >= RECOVER_OFFSET_M)
        wants_change = self.phase == "WAIT_GAP" or wants_return or wants_pass or wants_recover
        if wants_change and perception_ready and remaining > transition+20.:
            offsets = {0: 0.} if wants_return else {s: s*d for s, d in adjacent_offsets.items()}
            if wants_recover:
                offsets = {self.lane_side: self.target_offset}
                reason = f"back to the lane centre ({lateral-goal_offset:+.1f} m off)"
            if wants_pass:
                # Adjacent offsets are measured from the global route, not the
                # current lane. Return is one lane; opposite neighbours may be two.
                if abs(lateral) > 1.5:
                    offsets[0] = 0.
                offsets = {s: d for s, d in offsets.items() if 1.5 < abs(d-lateral) <= 5.5}
                reason = "checking further overtake after minimum lane-change goal"
            if wants_return:
                # Keep the first lane until an obstacle we passed clears the return lane.
                reason = "waiting for return lane gap while moving"
            # The default length follows the cruise target (112 m at 90 km/h).
            # A blocked lane at lower speed may need the shorter change that
            # the current speed allows at ~2 m/s^2 lateral (3.6 s, >= 60 m);
            # otherwise a short map corridor parks us behind a stopped car.
            # Only for an obstacle ahead in this lane within the long change's
            # reach; routine changes and lane ends (map) keep the gentle one.
            blocked_here = current_static < max(cruise_reserve, transition+40.)
            short = max(60., speed*3.6) if blocked_here else transition
            # Slow behind a blocked lane: a low-speed change (>= 15 m keeps
            # curvature near 0.09 1/m for a 3.5 m offset).
            creep = max(15., speed*3.6) if blocked_here and speed < 6. else short
            # Back to the route lane before the lane-change limit (the out lane is
            # closed by the exit-taper drum line from idx ~3045): shorter changes
            # and slower paces become acceptable as the limit approaches.
            urgent = ((wants_return or wants_recover or wants_pass and abs(lateral) > 1.5) and
                      change_room < transition + 60.)
            # Standing in a closed or blocked lane next to a queue: zipper merge at
            # walking pace into a gap between the queued cars.
            zipper = (wants_return or wants_recover) and speed < ZIPPER_SPEED_MPS
            quick = max(60., speed*3.) if urgent else transition
            if not wants_return and change_room < transition + 85.:
                # Late out change: a 3 s change at the current speed (>= 60 m, about
                # 2.3 m/s^2 peak lateral at 90 km/h) instead of the 112 m one, so a
                # short return still fits before the limit.
                quick = max(60., speed*3.)
            lengths = (transition, min(transition, short), min(transition, quick), min(transition, creep))
            if urgent and change_room < transition:
                # Late return: the short change first, so the lanes are crossed
                # before the taper's drum line between them (idx ~3045).
                lengths = (min(transition, quick),) + lengths
            lane_options = [(side, offset, length) for side, offset in offsets.items()
                            for length in dict.fromkeys(lengths)]
            # Try every lane at cruise pace before accepting a slower change.
            for cruise_only in (True, False):
                for side, offset, change_length in lane_options:
                    if self.saved_world is not None:
                        break
                    # Every out change needs a checked return before the taper,
                    # including overtakes after the minimum mission goal. Reserve
                    # the return length at current speed plus hold and gap-search room.
                    return_room = max(60., speed*3.)+25.+20.
                    if not (wants_return or wants_recover or abs(offset) < .5) and change_length + return_room > change_room:
                        candidate_checks.append(dict(offset_m=float(offset), transition_m=float(change_length),
                                                     accepted=False, reason="lane-change zone ends before the change"))
                        continue
                    candidate = lane_change_path(route, self.offsets_along(route, side, offset), change_length)
                    candidate_map = float(map_clearance(candidate, change_length+12.))
                    needed_map = change_length+10.
                    if wants_recover:
                        # We are the ones off the corridor (run 266: on the lane line); the
                        # way back only has to end in it: judge the path past the join.
                        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(candidate, axis=0), axis=1))]
                        settled = candidate[int(np.searchsorted(arc, change_length)):]
                        candidate_map = float(map_clearance(settled, 40.)) if len(settled) > 8 else 0.
                        needed_map = 30.
                    check = dict(offset_m=float(offset), map_clear_m=candidate_map,
                                 transition_m=float(change_length),
                                 accepted=False, reason="lane-change map corridor unavailable")
                    candidate_checks.append(check)
                    if not math.isfinite(candidate_map) or candidate_map < needed_map:
                        continue
                    if rear_unclassified is not None:
                        rear = np.asarray(rear_unclassified)
                        if len(rear) and np.any((rear[:, 0] < -2.5) & (rear[:, 0] > -70.) &
                                                # 1.2 m: a car in that lane, not the
                                                # roadside cones we just passed (1.4 m
                                                # off its centre kept us 130 m in the
                                                # out lane, sim highway_roadside_cones).
                                                (np.abs(rear[:, 1]-(offset-lateral)) < 1.2)):
                            reason = "unclassified rear returns; retain current lane"
                            check["reason"] = reason
                            continue
                    if any(o[4] == "npc" and len(o)>14 and not o[14] and
                           -70. < local_points(np.asarray(o[:2])[None, :], ego)[0, 0] < 5. and
                           abs(local_points(np.asarray(o[:2])[None, :], ego)[0, 1] - (offset-lateral)) < .946+float(o[7])/2.+.3
                           for o in objects):
                        reason = "rear velocity warmup; retain current lane"
                        check["reason"] = reason
                        continue
                    rear_block = target_lane_rear_conflict(ego, objects, offset-lateral, speed, zipper=zipper, body_shift_m=self.body_shift_m)
                    if rear_block is not None:
                        # Judged at our current speed, not at a planned slower pace: a
                        # 75 km/h car 7 m behind was "let pass" at a 42 km/h pace,
                        # then the cap rose again and it passed 0.2 m from our side
                        # (sim highway_adjacent_traffic, 2026-10-05).
                        check.update(reason="target-lane rear/side car too close", rear_conflict=rear_block)
                        continue
                    preferred_min = min(self.target_mps, 50./3.6)
                    paces = [self.target_mps*f for f in (1., .96, .92, .85, .7, .6)]
                    paces.append(preferred_min)
                    # Behind slow traffic the gap is usually at our own speed: keep it
                    # (or ease off) instead of only testing >= 50 km/h paces.
                    paces += [p for p in (speed, speed*.85) if 3. < p < preferred_min]
                    if urgent:
                        paces += [self.target_mps*f for f in (.45, .35)]
                    if zipper:
                        paces += [2., 1.]
                    # The rear gap is judged at the speed the target lane lets us keep:
                    # a return accepted at 25 km/h behind a 20 km/h lead had to slow in
                    # front of a 29 km/h car, which ran into our side (regression
                    # highway_random_3 in real time).
                    lane_cap = lead_vehicle_speed_cap(candidate, ego, objects, self.target_mps, self.body_shift_m)
                    if wants_return and not urgent and not zipper:
                        return_follow = self.longitudinal.preview(candidate, ego, objects, self.target_mps)
                        return_free = self.longitudinal.preview(candidate, ego, (), self.target_mps)
                        if lane_cap < self.target_mps-.5 or return_follow.speeds[-1] < return_free.speeds[-1]-.5:
                            check["reason"] = "keep overtaking lane until slower return-lane leader clears"
                            continue
                    if cruise_only:
                        paces = [self.target_mps]
                    for requested_pace in dict.fromkeys(paces):
                        pace = min(requested_pace, geometry_speed_cap(candidate, self.target_mps), lane_cap)
                        if cruise_only and pace < self.target_mps - .1:
                            continue
                        # Check the whole change: at 24 km/h the 112 m change takes 16 s,
                        # and with the fixed 7 s horizon the target lane was entered
                        # unchecked as a 40 km/h car came up beside us (regression
                        # highway_random_3 in real time).
                        horizon = min(18., max(7., change_length/max(min(pace, speed), 3.)+1.5))
                        candidate_risk = self.predicted_gap(candidate, ego, objects, pace, horizon_s=horizon, zipper=zipper)
                        if candidate_risk is None and pace > speed+1.:
                            # The gap must also hold if we do not gain the planned speed
                            # (headway, curve or ramp limits): a return planned at a
                            # faster pace met a 40 km/h car beside us at the actual
                            # 30 km/h (regression highway_random_3 in real time).
                            candidate_risk = self.predicted_gap(candidate, ego, objects, max(speed, 1.), horizon_s=horizon,
                                                       zipper=zipper)
                        if candidate_risk is not None:
                            risk = candidate_risk
                            check.update(reason="lane-change traffic gap occupied", gap_risk=candidate_risk)
                            continue
                        available = float(static_clearance(candidate, min(170., change_length+40.)))
                        check["static_clear_m"] = available
                        # New steering must retain room for an emergency stop.
                        required = speed*.4 + speed*speed/8. + 6.
                        # ... and never less than the standoff the stopping-reserve cap
                        # keeps: a 112 m change accepted at 24 km/h with 9 m clear past a
                        # stopped car parked us on it for good (sim highway_adjacent_traffic).
                        required = max(required, STATIC_STANDOFF_M + 3.)
                        if (wants_return or wants_pass) and not urgent and not zipper:
                            # Leaving a free lane is optional: only for a lane we can
                            # cruise in, never back behind the car we just passed.
                            required = max(required, cruise_reserve)
                        if not math.isfinite(available) or available < required:
                            check["reason"] = "lane-change static stopping reserve unavailable"
                            continue
                        self.change_origin = (self.lane_side, self.target_offset, self.phase)
                        self.saved_world = world_points(candidate, ego)
                        self.change_pace = pace
                        path, target, risk = candidate, pace, None
                        if wants_recover:
                            # Same lane, same phase: only the committed path is new.
                            self.recovering = True
                            reason = "back to the lane centre with checked rear gap" + (" (zipper)" if zipper else "")
                            check.update(accepted=True, reason=reason, target_kmh=pace*3.6)
                            break
                        self.target_offset = offset
                        self.lane_side = side
                        self.selected_side = side if not wants_return else self.selected_side
                        self.phase = "CHANGE_BACK" if wants_return else "CHANGE_OUT"
                        reason = "moving lane change with checked rear gap" + (" (zipper)" if zipper else "")
                        check.update(accepted=True, reason=reason, target_kmh=pace*3.6)
                        break
            beside = [c["rear_conflict"] for c in candidate_checks if c.get("rear_conflict")]
            if self.saved_world is None and beside and not wants_pass:
                # Gap matching: a car holding station beside us in the target lane
                # never opens a gap. Unless it is behind us and our lane lets us pull
                # away, ease to 2 m/s below its speed and merge in behind it (sim
                # highway_merge_slow_car_far: 30 km/h side by side for 200 m).
                block = min(beside, key=lambda b: b["speed_mps"])
                other = block["speed_mps"]
                pull_ahead = block["along_m"] < 0. and current_lead_cap > other+3.
                # Only when we are not already overtaking it.
                if not pull_ahead and other-2. < target and speed < other+3.:
                    target = max(0., other-2.)
                    gap_matching = True
            if self.saved_world is None:
                reason = ("no safe further overtake gap; retain current lane" if wants_pass else
                          "waiting for return lane gap while moving" if wants_return else
                          "waiting for lane-change gap while moving")
                if gap_matching:
                    reason = "lane-change gap matching: ease behind the car beside us"
                traffic_blocked = any(c.get("reason") == "lane-change traffic gap occupied"
                                      for c in candidate_checks)
                if urgent and traffic_blocked and target > RETURN_SEARCH_MPS:
                    # Let the route-lane traffic go by instead of driving on into
                    # the closed out lane.
                    target = RETURN_SEARCH_MPS
                    reason = "slow in the out lane for a lane-change return gap before the taper"
        elif wants_change and not perception_ready:
            reason = "LiDAR vehicle stream warming; retain current lane"
        elif wants_change and remaining <= transition+20.:
            reason = "insufficient highway distance for checked lane change"
        if self.saved_world is not None:
            path, replanned = self.unblock_change(route, path, ego, speed, map_clearance, static_clearance)
            if replanned:
                reason = replanned
        if self.saved_world is not None:
            current_risk = self.predicted_gap(path, ego, objects, target)
            if current_risk is not None:
                risk = current_risk
            if (current_risk is not None and current_risk.get("collision_when") is not None and
                    current_risk["collision_when"] < TRAFFIC_CONTROL_HORIZON_S):
                # Recheck mild speed adjustments without a crossing STOP rule.
                # Only for a predicted footprint conflict: a car closing from
                # behind is never a reason to slow down in front of it.
                # Fixed paces (never fractions of the current speed, which would
                # compound frame by frame), including the pace the change was
                # accepted at.
                retry = [self.target_mps*f for f in (.96, .92, .85)]
                retry.append(getattr(self, "change_pace", target))
                for pace in dict.fromkeys(p for p in retry if p < target):
                    if self.predicted_gap(path, ego, objects, pace) is None:
                        target = pace
                        risk = None
                        reason = "adjust moving lane-change timing"
                        break
        # Same-flow front vehicles use headway control, including before the
        # lane change is available. A car behind the current lane never asks
        # us to brake just because it is closing.
        headway_cap = lead_vehicle_speed_cap(path, ego, objects, self.target_mps, self.body_shift_m)
        if headway_cap < target:
            target = headway_cap
            reason = "moving lead-vehicle headway"
        side_cap, side_reason, side_info = float("inf"), None, None
        if self.saved_world is None:
            # During our own lane change the timed gap check owns the timing.
            side_cap, side_reason, side_info = self.side_traffic_speed_cap(path, ego, objects, speed, index)
        if side_cap < target:
            target = side_cap
            reason = side_reason
        map_available = float(map_clearance(path, 170.))
        static_available = float(static_clearance(path, 170.))
        map_available = map_available if math.isfinite(map_available) else 0.
        static_available = static_available if math.isfinite(static_available) else 0.
        available = min(map_available, static_available)
        # v*0.4 + v^2/(2*4) + body/reserve <= free distance. Behind a static
        # obstacle keep 12 m: from 5 m no lane change can steer around it, so a
        # car waiting for a neighbour-lane gap was trapped (virtual run).
        # Static returns whose distance keeps pace with us are a car the vehicle
        # detector has not listed (live 184/186/187: ~45 m ahead, held at
        # 50 km/h for 10-15 s). Brake relative to its measured speed; it is
        # masked as a vehicle once the detector lists it.
        mover = self.static_mover_speed(index, ego, static_available)
        # v*0.4 + (v^2 - v_lead^2)/(2*4) + standoff <= free distance (v_lead = 0
        # for anything not measured moving).
        map_cap = math.sqrt(1.6**2+8.*max(0., map_available-5.))-1.6
        static_cap = math.sqrt(1.6**2+8.*max(0., static_available-STATIC_STANDOFF_M)+mover*mover)-1.6
        reserve_cap = min(map_cap, static_cap)
        if map_available < static_available:
            # The map corridor is a lane rule, not an obstacle: it slows the
            # car while the path re-centres but never stops it on the highway
            # (live 2026-10-03: 0.7 m off centre -> corridor "2 m" -> parked).
            reserve_cap = min(static_cap, max(reserve_cap, HIGHWAY_MAP_FLOOR_MPS))
        if reserve_cap < target:
            target = reserve_cap
            reason = ("highway map corridor stopping reserve" if map_available <= static_available else
                      "highway moving obstacle (unlisted vehicle) stopping reserve" if mover > 0. else
                      "highway static obstacle stopping reserve")
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        curvature_values = path_curvature(path) if len(path)>3 else np.array([float("inf")])
        preview = arc[1:-1] <= max(25., speed*2.5)
        curvature = float(curvature_values[preview].max()) if preview.any() else float("inf")
        curve_cap = geometry_speed_cap(path, self.target_mps)
        if curve_cap < target:
            target = curve_cap
            reason = "highway curvature braking envelope"
        # The 60 km/h limit applies again from the end of A2256W000153 (rule
        # 3-2: 15 s penalty the moment it is exceeded). Arrive at or below
        # EXIT_SPEED at the mission end with the follower's 2.5 m/s^2.
        exit_cap = math.sqrt(EXIT_SPEED_MPS**2 + 2.*EXIT_DECEL_MPS2*max(0., remaining-EXIT_MARGIN_M))
        if exit_cap < target:
            target = exit_cap
            reason = "highway exit: below 60 km/h at the speed-limit boundary"
        predicted_emergency = False
        current_risk = self.predicted_gap(path, ego, objects, target,
                                lane_change=self.saved_world is not None)
        if current_risk is not None:
            risk = current_risk
            if current_risk.get("collision_when") is not None and current_risk["collision_when"] < 1.:
                predicted_emergency = True
                reason = "imminent predicted collision"
            elif (current_risk.get("collision_when") is not None and
                  current_risk["collision_when"] < TRAFFIC_CONTROL_HORIZON_S):
                # Highest pace below the current cap that clears every predicted
                # footprint (fall in behind a converging car, or keep the pace
                # that already lets it pass). Cutting 0.48 m/s per frame until
                # the conflict vanished overshot to below the lead's speed: 90 ->
                # 25 km/h behind a 30 km/h car, then a car behind closed in (sim
                # highway_adjacent_platoon; live runs 184/186/187 'predicted crossing').
                paced = next((pace for pace in (target*f for f in PACE_FACTORS)
                              if pace < target-.1 and
                              self.predicted_gap(path, ego, objects, pace,
                                        lane_change=self.saved_world is not None) is None), None)
                if paced is not None and paced >= speed-1.2*.4-1e-6:
                    target, risk = paced, None
                    reason = "pace for predicted crossing"
                else:
                    target = min(target, max(0., speed-1.2*.4) if paced is None else paced)
                    reason = "adjust speed for predicted crossing"
            elif current_risk.get("collision_when") is not None:
                # Keep distant predictions for lane selection; do not turn a
                # possible crossing several seconds away into an immediate dip.
                reason = "future traffic conflict: retain pace and check lane options"

        # Stop only inside the hard-braking (6 m/s^2) distance; beyond it the
        # stopping-reserve cap above already brakes. The 4 m/s^2 test stopped
        # the car at 65 km/h for a return 35.8 m ahead (live 2026-10-03).
        closing_sq = max(0., speed*speed-mover*mover)
        reserve_need = max(3., speed*.25+closing_sq/12.+3.)
        short_now = static_available <= reserve_need
        # One-frame LiDAR returns (a curve-edge drum 32 m ahead at 64 km/h,
        # live 2026-10-04) must be seen twice unless already inside an
        # 8 m/s^2 stop; the stopping-reserve cap still brakes meanwhile.
        hard = static_available <= max(3., speed*.25+closing_sq/16.+3.)
        emergency = predicted_emergency or (short_now and (hard or getattr(self, "_static_short_prev", False)))
        self._static_short_prev = short_now
        if emergency and not predicted_emergency:
            reason = ("highway map corridor emergency reserve" if map_available <= static_available else
                      "highway static obstacle emergency reserve")
        if (risk is not None and risk.get("collision_when") is not None and
                risk["collision_when"] < 1.0 and self.saved_world is not None):
            emergency = True
            reason = "imminent lane-change collision"
        final_target = 0. if emergency else max(0., target)
        self.status = dict(phase=self.phase, lane_changes=self.completed, lane_change_goal=2,
                           lane_change_goal_is_minimum=True, further_overtake_requested=bool(wants_pass),
                           candidate_checks=candidate_checks,
                           target_kmh=round(final_target*3.6, 1), cruise_kmh=self.target_mps*3.6,
                           minimum_speed_goal_kmh=min(50., self.target_mps*3.6),
                           speed_below_minimum_reason=reason if final_target*3.6 < min(50., self.target_mps*3.6)-.1 else None,
                           side_traffic=side_info,
                           speed_caps_kmh=dict(headway=headway_cap*3.6,
                                               side_traffic=side_cap*3.6 if math.isfinite(side_cap) else None,
                                               stopping_reserve=reserve_cap*3.6,
                                               curvature=curve_cap*3.6),
                           reason=reason, gap_risk=risk, offset_m=self.target_offset,
                           transition_m=transition, curvature_1pm=curvature,
                           steering_feedforward_deg=math.degrees(math.atan(3.90*curvature)),
                           clear_distance_m=available, static_mover_kmh=round(mover*3.6, 1),
                           clearance_components_m=dict(map=map_available, static=static_available),
                           perception_ready=perception_ready,
                           emergency_stop=bool(emergency))
        return dict(path=path, target_mps=final_target,
                    emergency=emergency, status=self.status)
