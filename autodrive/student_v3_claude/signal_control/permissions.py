"""Explicit route-signal permissions, independent of generic turn semantics."""

GREEN = "green"
RED_LEFT_ONLY = "red_left_only"
IGNORE = "ignore"
LEGACY = "maneuver"
POLICIES = (GREEN, RED_LEFT_ONLY, IGNORE, LEGACY)
FOURTH_SIGNAL_NODE = "A1256W000742"


def policy_for_line(node_id, maneuver):
    if maneuver == "right":
        return IGNORE
    return RED_LEFT_ONLY if node_id == FOURTH_SIGNAL_NODE else GREEN


def policy_allows(state, policy):
    if policy == IGNORE:
        return True
    if policy == GREEN:
        return state in ("GREEN", "GREEN_LEFT")
    if policy == RED_LEFT_ONLY:
        return state == "RED_LEFT"
    raise ValueError("unsupported explicit signal permission: " + str(policy))
