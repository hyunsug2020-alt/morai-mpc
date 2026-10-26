#!/usr/bin/env python3
"""Experimental planner entry point using an explicit clamped degree-5 B-spline.

The production planner remains unchanged. This entry point replaces only its
local offset-path generator, allowing an A/B launch against the same HD map,
object feed, controller, and vehicle parameters.
"""
import importlib.util
import math
import os
import sys

import rospy

_NODE_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if _NODE_DIRECTORY not in sys.path:
    sys.path.insert(0, _NODE_DIRECTORY)

from local_bypass_core import resample_polyline  # noqa: E402


_PRODUCTION_PATH = os.path.join(_NODE_DIRECTORY, "hdmap_lane_change.py")
_SPEC = importlib.util.spec_from_file_location(
    "hdmap_lane_change_production", _PRODUCTION_PATH)
production_planner = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(production_planner)

EPSILON = 1.0e-9


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


def _clamped_degree5_blend(value):
    """Bezier/B-spline span with controls [0, 0, 0, 1, 1, 1]."""
    u = max(0.0, min(1.0, float(value)))
    one_minus_u = 1.0 - u
    return (10.0 * one_minus_u ** 2 * u ** 3 +
            5.0 * one_minus_u * u ** 4 + u ** 5)


def build_clamped_bspline_offset_path(
        reference, target_offset, entry_end_s, hold_end_s, exit_end_s,
        start_offset=0.0, step=0.25):
    """Open clamped degree-5 B-spline bypass in Frenet coordinates."""
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
            weight = _clamped_degree5_blend(distance / entry_end_s)
            offset = start_offset + weight * (target_offset - start_offset)
        elif distance <= hold_end_s:
            offset = target_offset
        elif distance <= exit_end_s:
            weight = _clamped_degree5_blend(
                (distance - hold_end_s) / (exit_end_s - hold_end_s))
            offset = target_offset * (1.0 - weight)
        else:
            offset = 0.0
        heading = _point_heading(reference, index)
        output.append((
            point[0] - math.sin(heading) * offset,
            point[1] + math.cos(heading) * offset))
    return output


def main():
    production_planner.build_clamped_bspline_offset_path = (
        build_clamped_bspline_offset_path)
    planner = production_planner.HdmapLaneChange()
    rospy.logwarn(
        "[LaneChange] EXPERIMENTAL global CRUISE + obstacle-only open "
        "clamped degree-5 B-spline enabled")
    planner.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
