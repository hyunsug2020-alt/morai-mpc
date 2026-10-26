#!/usr/bin/env python3
"""Offline regression for SID 438; truth is used exclusively for scoring.

PYTHONPATH selects either the original ESKF or the development copy. All ROS
initialization and transport are mocked. Recorded clock diagnostics isolate
the recovery changes from the long-running clock estimator's unknown history.
"""

import argparse
import importlib.util
import json
import math
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import Mock, patch

import eskf
import numpy as np
import rosbag
import rospy
import yaml


ORIGIN = 1791306522.5858343
STOPS = ((104.683, 110.051), (127.448, 129.931))
RECOVERIES = ((283.547, 284.547), (287.319, 288.319))


def load_parameters(path, preset):
    parameters = {}
    for line in Path(path).read_text().splitlines():
        if line.startswith(" * /eskf_node/"):
            key, value = line.strip()[2:].split(": ", 1)
            if "..." not in value:
                parameters["~" + key.split("/")[-1]] = yaml.safe_load(value)
    parameters["~sensor_config_file"] = str(preset)
    return parameters


def load_truth(path):
    # Duplicate simulator timestamps are scored once, never fed to callbacks.
    samples = {}
    for line in Path(path).read_text().splitlines():
        item = json.loads(line)
        if item.get("type") == "truth_ego":
            samples[item["t_us"] / 1e6] = (item["x"], item["y"])
    stamps = sorted(samples)
    return np.array(stamps), np.array([samples[t] for t in stamps])


def metrics(samples, truth, recovery_intervals=RECOVERIES):
    stamps, positions = truth
    rows = np.asarray(samples, dtype=float)
    truth_xy = np.column_stack([
        np.interp(rows[:, 0], stamps, positions[:, axis]) for axis in (0, 1)])
    errors = np.linalg.norm(rows[:, 1:3] - truth_xy, axis=1)
    # Subtract actual truth displacement, not the estimator's velocity, to
    # ensure the limiter is not merely hiding a control-pose discontinuity.
    corrections = rows[:, 1:3] - truth_xy
    result = {"samples": len(rows), "recoveries": [], "stops": []}
    for start, end in recovery_intervals:
        indices = np.flatnonzero((rows[:, 0] - ORIGIN >= start)
                                 & (rows[:, 0] - ORIGIN <= end))
        maximum = 0.0
        violating = 0
        for offset, index in enumerate(indices):
            candidates = indices[offset + 1:]
            delta = rows[candidates, 0] - rows[index, 0]
            candidates = candidates[(delta >= 0.0) & (delta <= 0.5)]
            if len(candidates):
                distances = np.linalg.norm(corrections[candidates] - corrections[index], axis=1)
                maximum = max(maximum, float(np.max(distances)))
                violating += int(np.count_nonzero(distances > 2.0))
        after = (rows[:, 0] - ORIGIN >= end - 0.5) & (rows[:, 0] - ORIGIN <= end)
        result["recoveries"].append({
            "interval_s": [start, end], "correction_over_2m_pairs": violating,
            "max_correction_within_0_5s_m": maximum,
            "position_error_max_m": float(np.max(errors[indices])),
            "last_0_5s_error_median_m": float(np.median(errors[after])),
        })
    settled = (rows[:, 0] - ORIGIN >= 302.0) & (rows[:, 0] - ORIGIN <= 304.0)
    if np.any(settled):
        result["position_error_302_304s_median_m"] = float(np.median(errors[settled]))
    for start, end in STOPS:
        selected = (rows[:, 0] - ORIGIN >= start) & (rows[:, 0] - ORIGIN <= end)
        tail = selected & (rows[:, 0] - ORIGIN >= end - 0.5)
        result["stops"].append({
            "interval_s": [start, end],
            "max_abs_forward_speed_mps": float(np.max(np.abs(rows[selected, 3]))),
            "last_0_5s_max_abs_forward_speed_mps": float(np.max(np.abs(rows[tail, 3]))),
        })
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("truth", type=Path)
    parser.add_argument("--trace", type=Path)
    parser.add_argument("--correction-rate", type=float)
    args = parser.parse_args()
    source = Path(eskf.__file__).parent
    spec = importlib.util.spec_from_file_location("eskf_438_replay_node", source / "scripts/eskf_node.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    parameters = load_parameters(args.bag.parent / "eskf_localization.log",
                                 args.bag.parent / "sensor_preset.json")
    if args.correction_rate is not None:
        parameters["~publish_position_correction_rate_mps"] = args.correction_rate
    now = [0.0]
    outputs = []
    recorded = []
    output_only = []
    events = []
    debug = []
    with ExitStack() as stack:
        stack.enter_context(patch.object(module.rospy, "get_param", side_effect=(
            lambda key, default=None: parameters.get(key, default))))
        stack.enter_context(patch.object(module.rospy, "has_param", side_effect=(
            lambda key: key in parameters)))
        for name in ("init_node", "Publisher", "Subscriber", "Timer", "loginfo", "logwarn",
                     "loginfo_throttle", "logwarn_throttle"):
            stack.enter_context(patch.object(module.rospy, name))
        stack.enter_context(patch.object(module.tf2_ros, "TransformBroadcaster"))
        stack.enter_context(patch.object(module.rospy.Time, "now", side_effect=(
            lambda: module.rospy.Time.from_sec(now[0]))))
        node = module.ESKFNode()
        node.time_scale_source_mode = "pure_odometry"
        node._refresh_time_scale = lambda _: None

        def capture(message):
            outputs.append([message.header.stamp.to_sec(), message.pose.pose.position.x,
                            message.pose.pose.position.y, message.twist.twist.linear.x])
            if args.trace and any(a <= message.header.stamp.to_sec() - ORIGIN <= b
                                  for a, b in STOPS):
                debug.append([message.header.stamp.to_sec() - ORIGIN,
                              *node.last_imu_input.tolist(),
                              node.wheel_stationary_since if hasattr(node, "wheel_stationary_since") else None,
                              node.zupt_candidate_since,
                              getattr(node, "wheel_stationary_output_active", None),
                              message.twist.twist.linear.x])

        node.odom_pub = Mock()
        node.odom_pub.publish.side_effect = capture
        node.tf_broadcaster = None
        try:
            from eskf.output import PositionCorrectionLimiter
            recorded_limiter = PositionCorrectionLimiter(node.output_position.correction_rate)
        except ImportError:
            recorded_limiter = None
        previous_output_stamp = None
        with rosbag.Bag(str(args.bag)) as bag:
            callbacks = {"/imu/data": node.imu_callback, "/gps": node.gps_callback,
                         "/Competition_topic": node.vehicle_state_callback,
                         "/odometry/pure": node.odometry_callback}
            for topic, message, receipt in bag.read_messages(
                    topics=list(callbacks) + ["/eskf/diagnostics", "/eskf/odom"],
                    end_time=rospy.Time.from_sec(ORIGIN + 305.0)):
                now[0] = receipt.to_sec()
                if topic == "/eskf/diagnostics":
                    diagnostic = json.loads(message.data)
                    node.odometry_time_scale = diagnostic["odometry_time_scale"]
                elif topic == "/eskf/odom":
                    stamp = message.header.stamp.to_sec()
                    position = np.array([message.pose.pose.position.x, message.pose.pose.position.y])
                    forward = message.twist.twist.linear.x
                    recorded.append([stamp, *position, forward])
                    if recorded_limiter is not None:
                        q = message.pose.pose.orientation
                        yaw = module.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                        velocity = np.array([math.cos(yaw) * forward, math.sin(yaw) * forward])
                        stamp = max(stamp, previous_output_stamp or stamp)
                        delta = 0.0 if previous_output_stamp is None else stamp - previous_output_stamp
                        limited = recorded_limiter.update(
                            position, velocity, stamp, node._physics_duration(delta))
                        output_only.append([stamp, *limited, forward])
                        previous_output_stamp = stamp
                else:
                    previous_recovery = node.filter.counters["gps_reacquired"]
                    before = node.filter.base_state()[0] if node.filter.initialized else None
                    callbacks[topic](message)
                    if node.filter.counters["gps_reacquired"] > previous_recovery:
                        if hasattr(node, "gps_recovery_event") and node.gps_recovery_event:
                            events.append(dict(t=now[0] - ORIGIN, **node.gps_recovery_event))
                        elif before is not None:
                            distance = float(np.linalg.norm(node.filter.base_state()[0] - before))
                            events.append(dict(t=now[0] - ORIGIN, reason="legacy_recovery",
                                               internal_position_step_m=distance))
    truth = load_truth(args.truth)
    if args.trace:
        args.trace.write_text(json.dumps(dict(recorded=recorded, sensor_replay=outputs,
                                             output_only=output_only, debug=debug)))
    result = {"source": str(source), "recorded": metrics(recorded, truth),
              "sensor_replay": metrics(outputs, truth),
              "counters": dict(node.filter.counters), "recovery_events": events}
    moving_events = [e for e in events if 279.0 <= e["t"] <= 291.0 and (
        e["reason"] == "moving_internal_reanchor" or e.get("internal_position_step_m", 0.0) > 3.0)]
    if moving_events:
        result["sensor_reanchor_windows"] = metrics(
            outputs, truth, [(e["t"] - 0.5, e["t"] + 0.5) for e in moving_events])
    if output_only:
        result["recorded_pose_through_limiter"] = metrics(output_only, truth)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
