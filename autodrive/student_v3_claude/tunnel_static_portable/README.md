# Tunnel model + static obstacle avoidance

Portable extraction from MORAI `student_v3`. Python 3.8+ and NumPy required. Package has no ROS, camera, simulator, or model dependency; the host project supplies model predictions, perception objects, and lane/collision validation callbacks.

## Contents

- `tunnel.py`: route-bounded tunnel policy, exit rejoin phase, direct model-trajectory validation, and model-side hold state.
- `geometry.py`: curvature checks and vehicle-feasible S-curve construction.
- `static_obstacle.py`: static-object route checks, bypass selection, saved-path continuation, and speed cap helpers.
- `example_integration.py`: minimal host-loop example.

## Install

Copy `tunnel_static_portable/` into the target project and install NumPy. Import from its parent directory:

```python
from tunnel_static_portable import TunnelLaneHold, TunnelModelPolicy, TunnelRoutePolicy, plan_static_bypass
```

## Tunnel policy

`TunnelRoutePolicy` finds entry/exit indices by nearest XY point on a metric route. `state(nearest_route_index)` returns whether the vehicle is in the tunnel, whether centerline crossing may be enabled, whether rejoin must be deferred, and the rejoin start index. Rejoin phase starts at the configured distance before the exit (45 m by default). Apply `cross_center_allowed` only to tunnel collision geometry; keep ordinary-road lane rules unchanged.

`TunnelModelPolicy.plan(path, dt, action, probability)` passes a validated model path through unchanged and caps speed. It returns a zero-speed STOP state for invalid, reverse, discontinuous, stationary, or model-STOP paths. `path` must be an ego-local Nx2-or-wider array in metres; `dt` is the interval between predicted poses in seconds.

`TunnelLaneHold.observe_model(action, probability, state)` remembers model LEFT/RIGHT decisions before the exit rejoin zone. Call `observe_plan(plan, state)` after physical and lane validation; it clears the hold only when a checked `kind="rejoin"` plan succeeds near the exit. Pass the returned hold offset and direction into the static planner.

## Static obstacle planner

```python
plan, hazards = plan_static_bypass(
    dense_route, ego, objects,
    lane_ok=lane_ok,
    committed_direction=direction,
    previous_plan=previous_plan,
    adjacent_offsets=adjacent_offsets,
    corridor_clearance=corridor_clearance,
    defer_rejoin=tunnel_state["defer_rejoin"] and lane_hold is not None,
    deferred_offset_m=lane_hold["offset_m"] if lane_hold else 0.0,
    deferred_distance_m=tunnel_state["rejoin_distance_m"],
)
```

- `dense_route`: forward ego-local route Nx2+ in metres.
- `ego`: dict with world `x`, `y`, `heading_deg`, and `speed_kmh`.
- `objects`: iterable of tuples `(x, y, size, name, kind, speed, length, width, heading_deg, vx, vy, id, ...)`. Positions use the same world XY frame as `ego`. `kind` values used by the planner are `obstacle`, `pedestrian`, and `npc`; moving objects are excluded from static bypass planning.
- `lane_ok(path, tolerance_m)`: host callback; return true only if path segment lies in permitted drivable space. The planner trusts this callback to enforce one-way and centerline restrictions.
- `corridor_clearance(path, horizon_m)`: optional callback returning collision-free distance in metres. It should include vehicle footprint and any physical sensor checks.
- `adjacent_offsets`: mapping `{1: left_offset_m, -1: right_offset_m}` for connected legal lanes. Optional attributes `start_m_by_side` and `open_road_sides` select delayed joins. Pass `None` only when host intentionally allows in-lane geometric offsets without map lane metadata.
- `committed_direction`: `1` left, `-1` right, `0` uncommitted. Use model LEFT/RIGHT action as direction in the tunnel; collision and lane callbacks still validate the chosen path.
- `previous_plan`: previous planner result, retained by the host between frames. `continue_static_plan(saved, ego, now)` supports a host's world-frame saved plan; see source signature.
- Set `defer_rejoin=True` only after a bypass has committed. The planner holds the requested lateral offset and returns a blocked plan if no safe hold path validates; it will not silently substitute centerline rejoin.
- `STUDENT_BYPASS_ALAT` sets the lateral-acceleration limit used by the speed cap (default 1.7 m/s²).

Output is `(plan, hazards)`. A safe plan contains `path`, `direction`, `offset_m`, `curvature`, `speed_cap_mps`, clearance and replan fields. A blocked plan has `blocked=True`; stop or retain only a separately revalidated prior prefix. Never publish a candidate before the host's final lane and physical-sensor checks pass.

## Integration constraints

The extracted planner is a geometry component, not a full vehicle controller. The host must provide current, timestamped perception; a metric local route; same-direction legal lane offsets; complete vehicle-footprint collision checking; and a fail-safe stop when no path validates. Keep signal, pedestrian, dynamic-vehicle, stale-sensor, and emergency-stop logic in the host. Do not enable opposite-lane offsets outside the route-bounded tunnel.
