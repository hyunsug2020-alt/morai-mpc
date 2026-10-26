"""Host-loop sketch; replace callbacks and buffers with target-project code."""
from tunnel_static_portable import (
    TunnelLaneHold,
    TunnelModelPolicy,
    TunnelRoutePolicy,
    plan_static_bypass,
)


def configure(route_world_xy, tunnel_entry_xy, tunnel_exit_xy):
    route_scope = TunnelRoutePolicy(
        route_world_xy, tunnel_entry_xy, tunnel_exit_xy, rejoin_buffer_m=45.0, enabled=True
    )
    model = TunnelModelPolicy(
        route_world_xy, tunnel_entry_xy, tunnel_exit_xy, max_kmh=50.0, enabled=True
    )
    return route_scope, model, TunnelLaneHold()


def on_frame(route_scope, model, nearest_route_index, model_path, dt, action, probability,
             dense_route_local, ego, objects, lane_ok, corridor_clearance,
             lane_hold_state, committed_direction, previous_plan, adjacent_offsets):
    state = route_scope.state(nearest_route_index)
    lane_hold = lane_hold_state.observe_model(action, probability, state)
    path = dense_route_local
    model_info = {"control_owner": "host route", "motion_state": "DRIVE"}
    if state["tunnel_active"]:
        path, model_info = model.plan(model_path, dt, action, probability)
        if model_info["motion_state"] == "STOP":
            return path, model_info, None, state

    # Host should set/update lane_hold from a validated model lane-change decision.
    defer = bool(state["defer_rejoin"] and lane_hold is not None)
    if lane_hold:
        committed_direction = lane_hold["direction"]
    plan, hazards = plan_static_bypass(
        dense_route_local, ego, objects, lane_ok,
        committed_direction=committed_direction,
        previous_plan=previous_plan,
        adjacent_offsets=adjacent_offsets,
        corridor_clearance=corridor_clearance,
        defer_rejoin=defer,
        deferred_offset_m=lane_hold["offset_m"] if lane_hold else 0.0,
        deferred_distance_m=state["rejoin_distance_m"],
    )
    if plan and plan.get("blocked"):
        model_info = dict(model_info, motion_state="STOP", stop_reason="no safe static bypass")
        return path, model_info, (plan, hazards), state
    lane_hold_state.observe_plan(plan, state)
    if plan is not None:
        path = plan["path"]
    return path, model_info, (plan, hazards), state
