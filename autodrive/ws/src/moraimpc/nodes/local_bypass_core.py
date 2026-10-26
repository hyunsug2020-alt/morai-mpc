#!/usr/bin/env python3
"""ROS-independent geometry primitives for constrained local bypass planning.

All angles are radians and all distances are metres.  The module deliberately
contains no ROS imports so candidate generation and safety checks can be replayed
from recorded data in ordinary unit tests.
"""

from dataclasses import dataclass
import json
import math


EPSILON = 1.0e-9
AREA_FORMAT = "morai_intersection_drivable_areas"


@dataclass(frozen=True)
class DrivableArea:
    area_id: str
    drivable_polygon: tuple
    forbidden_polygons: tuple = ()
    enabled: bool = True


@dataclass(frozen=True)
class FrenetProjection:
    s: float
    d: float
    x: float
    y: float
    heading: float
    segment_index: int
    fraction: float
    distance: float


@dataclass(frozen=True)
class ObstacleOBB:
    x: float
    y: float
    yaw: float
    length: float
    width: float
    vx: float = 0.0
    vy: float = 0.0
    kind: str = "Obstacle"
    object_id: object = None


def _finite_point(raw):
    if not isinstance(raw, (list, tuple)) or len(raw) < 2:
        raise ValueError("polygon point must contain x and y")
    point = (float(raw[0]), float(raw[1]))
    if not all(math.isfinite(value) for value in point):
        raise ValueError("polygon point must be finite")
    return point


def signed_area(points):
    return 0.5 * sum(
        first[0] * second[1] - second[0] * first[1]
        for first, second in zip(points, points[1:] + points[:1]))


def _orientation(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1]) -
            (b[1] - a[1]) * (c[0] - a[0]))


def _on_segment(a, b, point, epsilon=EPSILON):
    return (
        min(a[0], b[0]) - epsilon <= point[0] <=
        max(a[0], b[0]) + epsilon and
        min(a[1], b[1]) - epsilon <= point[1] <=
        max(a[1], b[1]) + epsilon and
        abs(_orientation(a, b, point)) <= epsilon)


def segments_intersect(a, b, c, d, include_touch=True):
    ab_c = _orientation(a, b, c)
    ab_d = _orientation(a, b, d)
    cd_a = _orientation(c, d, a)
    cd_b = _orientation(c, d, b)
    proper = (
        ((ab_c > EPSILON and ab_d < -EPSILON) or
         (ab_c < -EPSILON and ab_d > EPSILON)) and
        ((cd_a > EPSILON and cd_b < -EPSILON) or
         (cd_a < -EPSILON and cd_b > EPSILON)))
    if proper or not include_touch:
        return proper
    return (
        (abs(ab_c) <= EPSILON and _on_segment(a, b, c)) or
        (abs(ab_d) <= EPSILON and _on_segment(a, b, d)) or
        (abs(cd_a) <= EPSILON and _on_segment(c, d, a)) or
        (abs(cd_b) <= EPSILON and _on_segment(c, d, b)))


def validate_polygon(points, minimum_area_m2=1.0):
    """Return ``(valid, reason)`` for a simple, non-degenerate polygon."""
    try:
        polygon = [_finite_point(point) for point in points]
    except (TypeError, ValueError) as exc:
        return False, str(exc)
    if len(polygon) < 3:
        return False, "polygon needs at least three points"
    if len(set((round(x, 9), round(y, 9)) for x, y in polygon)) < 3:
        return False, "polygon needs at least three distinct points"
    count = len(polygon)
    for first in range(count):
        a, b = polygon[first], polygon[(first + 1) % count]
        if math.hypot(b[0] - a[0], b[1] - a[1]) <= EPSILON:
            return False, "polygon contains a zero-length edge"
        for second in range(first + 1, count):
            if len({first, (first + 1) % count,
                    second, (second + 1) % count}) < 4:
                continue
            c, d = polygon[second], polygon[(second + 1) % count]
            if segments_intersect(a, b, c, d):
                return False, "polygon edges intersect"
    area = abs(signed_area(polygon))
    if area < float(minimum_area_m2):
        return False, "polygon area %.3f is below %.3f" % (
            area, minimum_area_m2)
    return True, ""


def load_drivable_areas(path, minimum_area_m2=1.0):
    """Load and strictly validate an intersection corridor JSON file."""
    with open(path, encoding="utf-8") as stream:
        payload = json.load(stream)
    if not isinstance(payload, dict) or payload.get("format") != AREA_FORMAT:
        raise ValueError("unexpected intersection area format")
    if payload.get("coordinate_frame", "map") != "map":
        raise ValueError("intersection areas must use the map frame")
    raw_areas = payload.get("intersections", [])
    if not isinstance(raw_areas, list):
        raise ValueError("intersections must be a list")
    areas = []
    identifiers = set()
    for index, raw in enumerate(raw_areas):
        if not isinstance(raw, dict):
            raise ValueError("intersection entry must be an object")
        area_id = str(raw.get("id") or "intersection_%03d" % (index + 1))
        if area_id in identifiers:
            raise ValueError("duplicate intersection id: %s" % area_id)
        identifiers.add(area_id)
        outer = tuple(_finite_point(point)
                      for point in raw.get("drivable_polygon", []))
        valid, reason = validate_polygon(outer, minimum_area_m2)
        if not valid:
            raise ValueError("%s drivable polygon: %s" % (area_id, reason))
        forbidden = []
        for polygon_index, raw_polygon in enumerate(
                raw.get("forbidden_polygons", [])):
            polygon = tuple(_finite_point(point) for point in raw_polygon)
            valid, reason = validate_polygon(polygon, minimum_area_m2)
            if not valid:
                raise ValueError("%s forbidden polygon %d: %s" % (
                    area_id, polygon_index, reason))
            forbidden.append(polygon)
        areas.append(DrivableArea(
            area_id=area_id,
            drivable_polygon=outer,
            forbidden_polygons=tuple(forbidden),
            enabled=bool(raw.get("enabled", True))))
    return areas


def point_in_polygon(point, polygon, include_boundary=True):
    """Even-odd point-in-polygon test supporting concave simple polygons."""
    if len(polygon) < 3:
        return False
    inside = False
    px, py = point
    previous = polygon[-1]
    for current in polygon:
        if _on_segment(previous, current, point):
            return include_boundary
        x1, y1 = previous
        x2, y2 = current
        if (y1 > py) != (y2 > py):
            crossing_x = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
            if crossing_x > px:
                inside = not inside
        previous = current
    return inside


def polygon_boundaries_intersect(first, second, include_touch=True):
    for a, b in zip(first, first[1:] + first[:1]):
        for c, d in zip(second, second[1:] + second[:1]):
            if segments_intersect(a, b, c, d, include_touch=include_touch):
                return True
    return False


def polygons_overlap(first, second):
    return (
        polygon_boundaries_intersect(first, second) or
        point_in_polygon(first[0], second) or
        point_in_polygon(second[0], first))


def obb_corners(x, y, yaw, length, width, inflation=0.0):
    half_length = 0.5 * max(0.0, float(length)) + float(inflation)
    half_width = 0.5 * max(0.0, float(width)) + float(inflation)
    if half_length <= 0.0 or half_width <= 0.0:
        raise ValueError("OBB length and width must be positive")
    forward = (math.cos(yaw), math.sin(yaw))
    left = (-math.sin(yaw), math.cos(yaw))
    return tuple((
        x + sx * half_length * forward[0] + sy * half_width * left[0],
        y + sx * half_length * forward[1] + sy * half_width * left[1],
    ) for sx, sy in ((1.0, 1.0), (1.0, -1.0),
                     (-1.0, -1.0), (-1.0, 1.0)))


def footprint_inside_area(x, y, yaw, length, width, area, margin=0.0):
    """Check the whole inflated ego rectangle against area and exclusions."""
    if not isinstance(area, DrivableArea) or not area.enabled:
        return False
    footprint = obb_corners(x, y, yaw, length, width, inflation=margin)
    outer = area.drivable_polygon
    if not all(point_in_polygon(point, outer, include_boundary=False)
               for point in footprint):
        return False
    # Corners alone are insufficient for a concave outer polygon.
    if polygon_boundaries_intersect(footprint, outer):
        return False
    return not any(polygons_overlap(footprint, forbidden)
                   for forbidden in area.forbidden_polygons)


def resample_polyline(points, step=0.5):
    """Resample a polyline at approximately constant arc-length spacing."""
    if step <= 0.0:
        raise ValueError("step must be positive")
    points = [_finite_point(point) for point in points]
    if len(points) < 2:
        return points
    clean = [points[0]]
    for point in points[1:]:
        if math.hypot(point[0] - clean[-1][0],
                      point[1] - clean[-1][1]) > EPSILON:
            clean.append(point)
    if len(clean) < 2:
        return clean
    cumulative = [0.0]
    for first, second in zip(clean, clean[1:]):
        cumulative.append(cumulative[-1] + math.hypot(
            second[0] - first[0], second[1] - first[1]))
    total = cumulative[-1]
    targets = []
    distance = 0.0
    while distance < total:
        targets.append(distance)
        distance += step
    targets.append(total)
    output = []
    segment = 0
    for target in targets:
        while (segment + 1 < len(cumulative) - 1 and
               cumulative[segment + 1] < target):
            segment += 1
        span = cumulative[segment + 1] - cumulative[segment]
        fraction = 0.0 if span <= EPSILON else (
            target - cumulative[segment]) / span
        first, second = clean[segment], clean[segment + 1]
        output.append((
            first[0] + fraction * (second[0] - first[0]),
            first[1] + fraction * (second[1] - first[1])))
    return output


def project_to_frenet(px, py, reference):
    """Project a point onto a polyline and return signed Frenet coordinates."""
    if len(reference) < 2:
        raise ValueError("reference needs at least two points")
    travelled = 0.0
    best = None
    for index, (start, end) in enumerate(zip(reference, reference[1:])):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= EPSILON:
            continue
        fraction = max(0.0, min(
            1.0, ((px - start[0]) * dx + (py - start[1]) * dy) /
            (length * length)))
        x = start[0] + fraction * dx
        y = start[1] + fraction * dy
        heading = math.atan2(dy, dx)
        d = -math.sin(heading) * (px - x) + math.cos(heading) * (py - y)
        distance = math.hypot(px - x, py - y)
        candidate = FrenetProjection(
            s=travelled + fraction * length, d=d, x=x, y=y,
            heading=heading, segment_index=index, fraction=fraction,
            distance=distance)
        if best is None or candidate.distance < best.distance:
            best = candidate
        travelled += length
    if best is None:
        raise ValueError("reference contains no non-zero segment")
    return best


def obb_projected_half_extents(length, width, object_yaw, route_yaw):
    """Return OBB half extents along and normal to a route tangent."""
    delta = object_yaw - route_yaw
    along = 0.5 * (abs(length * math.cos(delta)) +
                   abs(width * math.sin(delta)))
    lateral = 0.5 * (abs(length * math.sin(delta)) +
                     abs(width * math.cos(delta)))
    return along, lateral


def sample_lateral_targets(object_d, object_lateral_half_extent,
                           ego_half_width, clearance, tracking_margin,
                           minimum_offset, maximum_offset, step=0.2):
    """Sample minimal-to-larger collision-free offsets on both object sides."""
    if step <= 0.0 or minimum_offset >= maximum_offset:
        raise ValueError("invalid lateral sampling bounds")
    separation = (float(object_lateral_half_extent) + float(ego_half_width) +
                  float(clearance) + float(tracking_margin))
    left_start = float(object_d) + separation
    right_start = float(object_d) - separation
    targets = []
    value = left_start
    while value <= maximum_offset + EPSILON:
        if value >= minimum_offset - EPSILON:
            targets.append(value)
        value += step
    value = right_start
    while value >= minimum_offset - EPSILON:
        if value <= maximum_offset + EPSILON:
            targets.append(value)
        value -= step
    return sorted(set(round(value, 9) for value in targets),
                  key=lambda value: (abs(value), value))


def quintic_smoothstep(value):
    value = max(0.0, min(1.0, float(value)))
    return value ** 3 * (value * (value * 6.0 - 15.0) + 10.0)


def minimum_quintic_transition_length(offset_change, max_curvature,
                                      max_heading_change=math.radians(20.0)):
    """Conservative straight-reference length bound for a quintic shift."""
    distance = abs(float(offset_change))
    if distance <= EPSILON:
        return 0.0
    if max_curvature <= 0.0 or not (0.0 < max_heading_change < math.pi / 2.0):
        raise ValueError("curvature and heading limits must be positive")
    curvature_bound = math.sqrt(5.773502691896258 * distance /
                                max_curvature)
    heading_bound = 1.875 * distance / math.tan(max_heading_change)
    return max(curvature_bound, heading_bound)


def _polyline_arc(points):
    arc = [0.0]
    for first, second in zip(points, points[1:]):
        arc.append(arc[-1] + math.hypot(
            second[0] - first[0], second[1] - first[1]))
    return arc


def _point_heading(points, index):
    before = max(0, index - 1)
    after = min(len(points) - 1, index + 1)
    return math.atan2(points[after][1] - points[before][1],
                      points[after][0] - points[before][0])


def path_headings(path):
    """Return a centered finite-difference heading for every path point."""
    if len(path) < 2:
        raise ValueError("path needs at least two points")
    return [_point_heading(path, index) for index in range(len(path))]


def build_clamped_bspline_offset_path(
        reference, target_offset, entry_end_s, hold_end_s, exit_end_s,
        start_offset=0.0, step=0.25):
    """Build an open clamped degree-5 B-spline bypass in Frenet space.

    Each transition is the single-span B-spline/Bezier form with repeated
    endpoint controls, giving zero lateral velocity and acceleration at entry,
    hold, and global-path rejoin.
    """
    reference = resample_polyline(reference, step)
    if len(reference) < 2:
        raise ValueError("reference needs at least two distinct points")
    entry_end_s = float(entry_end_s)
    hold_end_s = float(hold_end_s)
    exit_end_s = float(exit_end_s)
    if not (entry_end_s > 0.0 and
            entry_end_s <= hold_end_s < exit_end_s):
        raise ValueError("expected 0 < entry <= hold < exit")
    arc = _polyline_arc(reference)
    if arc[-1] + EPSILON < exit_end_s:
        raise ValueError("reference ends before bypass rejoin")
    output = []
    for index, point in enumerate(reference):
        distance = arc[index]
        if distance <= entry_end_s:
            weight = quintic_smoothstep(distance / entry_end_s)
            offset = start_offset + weight * (target_offset - start_offset)
        elif distance <= hold_end_s:
            offset = target_offset
        elif distance <= exit_end_s:
            weight = quintic_smoothstep(
                (distance - hold_end_s) / (exit_end_s - hold_end_s))
            offset = target_offset * (1.0 - weight)
        else:
            offset = 0.0
        heading = _point_heading(reference, index)
        output.append((
            point[0] - math.sin(heading) * offset,
            point[1] + math.cos(heading) * offset))
    return output


def build_quintic_offset_path(reference, target_offset, entry_end_s,
                               hold_end_s, exit_end_s, start_offset=0.0,
                               step=0.25):
    """Compatibility wrapper for the clamped degree-5 B-spline generator."""
    return build_clamped_bspline_offset_path(
        reference, target_offset, entry_end_s, hold_end_s, exit_end_s,
        start_offset=start_offset, step=step)


def _axis_projection(polygon, axis):
    values = [point[0] * axis[0] + point[1] * axis[1]
              for point in polygon]
    return min(values), max(values)


def _sat_penetration(first, second):
    minimum_overlap = float("inf")
    for polygon in (first, second):
        for start, end in zip(polygon, polygon[1:] + polygon[:1]):
            edge = (end[0] - start[0], end[1] - start[1])
            length = math.hypot(*edge)
            if length <= EPSILON:
                continue
            axis = (-edge[1] / length, edge[0] / length)
            first_min, first_max = _axis_projection(first, axis)
            second_min, second_max = _axis_projection(second, axis)
            overlap = min(first_max, second_max) - max(first_min, second_min)
            if overlap < -EPSILON:
                return None
            minimum_overlap = min(minimum_overlap, max(0.0, overlap))
    return minimum_overlap


def _point_segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= EPSILON:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    fraction = max(0.0, min(
        1.0, ((point[0] - start[0]) * dx +
              (point[1] - start[1]) * dy) / length_sq))
    projection = (start[0] + fraction * dx, start[1] + fraction * dy)
    return math.hypot(point[0] - projection[0],
                      point[1] - projection[1])


def polygon_signed_clearance(first, second):
    """Euclidean separation, or negative SAT penetration when overlapping."""
    penetration = _sat_penetration(first, second)
    if penetration is not None:
        return -penetration
    distances = []
    for point in first:
        distances.extend(_point_segment_distance(point, start, end)
                         for start, end in zip(second, second[1:] + second[:1]))
    for point in second:
        distances.extend(_point_segment_distance(point, start, end)
                         for start, end in zip(first, first[1:] + first[:1]))
    return min(distances)


def obb_signed_clearance(first, second):
    first_polygon = obb_corners(
        first.x, first.y, first.yaw, first.length, first.width)
    second_polygon = obb_corners(
        second.x, second.y, second.yaw, second.length, second.width)
    return polygon_signed_clearance(first_polygon, second_polygon)


def _coerce_obstacle(raw):
    if isinstance(raw, ObstacleOBB):
        return raw
    if isinstance(raw, dict):
        return ObstacleOBB(
            x=float(raw["x"]), y=float(raw["y"]),
            yaw=float(raw.get("yaw", 0.0)),
            length=float(raw.get("length", 1.0)),
            width=float(raw.get("width", 1.0)),
            vx=float(raw.get("vx", 0.0)), vy=float(raw.get("vy", 0.0)),
            kind=str(raw.get("kind", "Obstacle")),
            object_id=raw.get("object_id"))
    raise TypeError("obstacle must be ObstacleOBB or dict")


def swept_obb_clearance(path, obstacles, ego_length, ego_width,
                        speed_mps, safety_margin=0.0, sample_step=0.2):
    """Check dense ego poses against constant-velocity predicted object OBBs."""
    dense = resample_polyline(path, sample_step)
    if len(dense) < 2:
        raise ValueError("path needs at least two distinct points")
    speed = max(0.1, abs(float(speed_mps)))
    arc = _polyline_arc(dense)
    minimum = float("inf")
    collision = None
    for index, point in enumerate(dense):
        yaw = _point_heading(dense, index)
        ego = ObstacleOBB(
            point[0], point[1], yaw, ego_length, ego_width)
        time_s = arc[index] / speed
        for raw in obstacles:
            obstacle = _coerce_obstacle(raw)
            predicted = ObstacleOBB(
                obstacle.x + obstacle.vx * time_s,
                obstacle.y + obstacle.vy * time_s,
                obstacle.yaw, obstacle.length, obstacle.width,
                obstacle.vx, obstacle.vy, obstacle.kind,
                obstacle.object_id)
            clearance = obb_signed_clearance(ego, predicted) - safety_margin
            if clearance < minimum:
                minimum = clearance
                collision = {
                    "path_index": index,
                    "time_s": time_s,
                    "object_id": obstacle.object_id,
                    "kind": obstacle.kind,
                    "clearance": clearance,
                }
    return {
        "valid": minimum >= 0.0,
        "minimum_clearance": minimum,
        "closest": collision,
    }


def path_curvatures(path):
    if len(path) < 3:
        return [0.0] * len(path)
    values = [None]
    for first, middle, last in zip(path, path[1:], path[2:]):
        ab = math.hypot(middle[0] - first[0], middle[1] - first[1])
        bc = math.hypot(last[0] - middle[0], last[1] - middle[1])
        ca = math.hypot(first[0] - last[0], first[1] - last[1])
        denominator = ab * bc * ca
        if denominator <= EPSILON:
            values.append(0.0)
            continue
        cross = ((middle[0] - first[0]) * (last[1] - first[1]) -
                 (middle[1] - first[1]) * (last[0] - first[0]))
        values.append(2.0 * cross / denominator)
    values.append(None)
    # Endpoints have no centered three-point estimate. Reuse the nearest
    # physical estimate instead of inventing a zero-curvature steering step.
    values[0] = values[1]
    values[-1] = values[-2]
    return values


def validate_path_kinematics(path, wheelbase, max_steer_deg,
                             max_steer_rate_deg_s, speed_mps,
                             max_lateral_accel_mps2,
                             max_operating_curvature=None):
    """Validate curvature, steering, steer rate and lateral acceleration."""
    if len(path) < 3:
        raise ValueError("path needs at least three points")
    if wheelbase <= 0.0 or max_steer_deg <= 0.0 or speed_mps < 0.0:
        raise ValueError("invalid vehicle kinematic limits")
    curvature = path_curvatures(path)
    steering = [math.atan(wheelbase * value) for value in curvature]
    physical_curvature = math.tan(math.radians(max_steer_deg)) / wheelbase
    allowed_curvature = physical_curvature
    if max_operating_curvature is not None:
        allowed_curvature = min(allowed_curvature,
                                float(max_operating_curvature))
    maximum_curvature = max(abs(value) for value in curvature)
    maximum_steer = max(abs(math.degrees(value)) for value in steering)
    maximum_rate = 0.0
    speed = max(0.0, float(speed_mps))
    if speed > EPSILON:
        for index in range(1, len(path)):
            distance = math.hypot(
                path[index][0] - path[index - 1][0],
                path[index][1] - path[index - 1][1])
            if distance <= EPSILON:
                continue
            dt = distance / speed
            rate = abs(math.degrees(
                steering[index] - steering[index - 1])) / dt
            maximum_rate = max(maximum_rate, rate)
    lateral_accel = speed * speed * maximum_curvature
    speed_cap = float("inf")
    if maximum_curvature > EPSILON:
        speed_cap = math.sqrt(max(0.0, max_lateral_accel_mps2) /
                              maximum_curvature)
    reasons = []
    if maximum_curvature > allowed_curvature + EPSILON:
        reasons.append("curvature")
    if maximum_steer > max_steer_deg + EPSILON:
        reasons.append("steer")
    if maximum_rate > max_steer_rate_deg_s + EPSILON:
        reasons.append("steer_rate")
    if lateral_accel > max_lateral_accel_mps2 + EPSILON:
        reasons.append("lateral_accel")
    return {
        "valid": not reasons,
        "reasons": tuple(reasons),
        "max_curvature": maximum_curvature,
        "max_steer_deg": maximum_steer,
        "max_steer_rate_deg_s": maximum_rate,
        "lateral_accel_mps2": lateral_accel,
        "speed_cap_mps": speed_cap,
    }


def candidate_score(target_offset, minimum_clearance, max_curvature,
                    path_length, weights=None):
    """Deterministic lower-is-better score for already valid candidates."""
    if minimum_clearance < 0.0:
        return float("inf")
    weights = weights or {}
    excursion_weight = float(weights.get("excursion", 1.0))
    clearance_weight = float(weights.get("clearance", 2.0))
    curvature_weight = float(weights.get("curvature", 20.0))
    length_weight = float(weights.get("length", 0.002))
    return (
        excursion_weight * abs(float(target_offset)) +
        clearance_weight / max(0.05, float(minimum_clearance) + 0.05) +
        curvature_weight * abs(float(max_curvature)) +
        length_weight * max(0.0, float(path_length)))
