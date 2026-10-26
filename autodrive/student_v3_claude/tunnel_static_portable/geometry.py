"""Vehicle-feasible planar bypass geometry, extracted for reuse."""
import numpy as np

MAX_CONTROL_CURVATURE = float(np.tan(np.radians(35.0)) / 3.0)

def path_curvatures(points):
    """Discrete planar curvature in 1/m for path feasibility checks."""
    points = np.asarray(points, dtype=np.float64)[:, :2]
    if len(points) < 3:
        return np.zeros(0, dtype=np.float64)
    ab, bc = points[1:-1]-points[:-2], points[2:]-points[1:-1]
    ac = points[2:]-points[:-2]
    denominator = (np.linalg.norm(ab, axis=1)*np.linalg.norm(bc, axis=1)*
                   np.linalg.norm(ac, axis=1))
    cross = np.abs(ab[:, 0]*ac[:, 1]-ab[:, 1]*ac[:, 0])
    return np.divide(2.*cross, denominator, out=np.zeros(len(cross)),
                     where=~(denominator < 1e-8))

def build_forced_bypass(dense_route, direction, offset_m=2.2, hold_m=6.0,
                        path_length_m=60.0, max_curvature_1pm=0.16, transitions=(10.0, 12.0, 14.0, 16.0, 18.0, 20.0),
                        start_m=0.0):
    """Build a vehicle-feasible S-curve using only Student's avoidance direction."""
    route = np.asarray(dense_route, dtype=np.float32)
    route_xy = route[:, :2]
    route_s = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(route_xy, axis=0), axis=1))))
    route_s, unique = np.unique(route_s, return_index=True)
    route_xy = route_xy[unique]
    usable = route_s <= min(float(path_length_m), float(route_s[-1]))
    route_s, map_base = route_s[usable], route_xy[usable]
    if len(route_s) < 3 or route_s[-1] < 25.0:
        return None, float("inf"), 0.0

    tangent = np.gradient(map_base, route_s, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-6)
    normals = np.column_stack((-tangent[:, 1], tangent[:, 0]))

    def quintic(value):
        value = np.clip(value, 0.0, 1.0)
        return value ** 3 * (10.0 - 15.0 * value + 6.0 * value ** 2)

    best = None
    for transition_m in transitions:
        return_start = float(start_m) + transition_m + hold_m
        profile = float(direction) * float(offset_m) * (1.0 - quintic(
            (route_s - return_start) / transition_m))
        # Offset the map lane first, then join to it from the actual pose and
        # heading. Offsetting the normals of an already joined path makes a
        # second S bend when ego is already in the adjacent lane.
        reference = map_base + normals * profile[:, None]
        initial_tangent = (reference[1] - reference[0]) / (route_s[1] - route_s[0])
        correction = reference[0] + route_s[:, None] * (initial_tangent - np.array([1., 0.]))
        candidate = np.zeros((len(reference), 3), dtype=np.float32)
        candidate[:, :2] = reference - correction * (1.0 - quintic(route_s / transition_m))[:, None]
        if start_m > 0.0:
            # A neighbouring tunnel lane may become legal only further ahead.
            # Join the route now, then begin the lateral manoeuvre there.
            initial_tangent = (map_base[1] - map_base[0]) / (route_s[1] - route_s[0])
            correction = map_base[0] + route_s[:, None] * (initial_tangent - np.array([1., 0.]))
            joined = map_base - correction * (1.0 - quintic(
                route_s / min(6.0, float(start_m))))[:, None]
            profile *= quintic((route_s - float(start_m)) / transition_m)
            candidate[:, :2] = joined + normals * profile[:, None]
        curvature = path_curvatures(candidate)
        peak_curvature = float(np.max(curvature)) if len(curvature) else float("inf")
        if best is None or peak_curvature < best[1]:
            best = (candidate, peak_curvature, transition_m)
        if peak_curvature <= max_curvature_1pm:
            return candidate, peak_curvature, transition_m
    return best
