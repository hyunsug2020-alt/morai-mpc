#!/usr/bin/env python3
"""SID 449 sensor-only replay; truth is loaded after replay for scoring.

No ROS master, publishers or live processes are used. GPS before 640 s warms
the filter; every GPS message at/after 640 s is explicitly excluded. Recorded
clock diagnostics supply the clock ratio established before bag recording.
Recorded ESKF poses are comparison data, never state initialization/input.
"""

import argparse
import importlib.util
import json
from collections import Counter
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import Mock, patch

import eskf
import numpy as np
import rosbag
import rospy

from replay_438 import load_parameters, load_truth


ORIGIN = 1791313823.498270
START, END = 640.0, 738.0


def score(rows, truth):
    rows = np.asarray(rows)
    rows = rows[(rows[:, 0] >= ORIGIN + START) & (rows[:, 0] <= ORIGIN + END)]
    # Collapse duplicate output stamps so the endpoint is the final correction
    # at that stamp. Interpolate truth at these exact sensor output times.
    indices = np.r_[np.flatnonzero(np.diff(rows[:, 0]) != 0.0), len(rows) - 1]
    rows = rows[indices]
    stamps, xy = truth
    reference = np.column_stack([np.interp(rows[:, 0], stamps, xy[:, axis])
                                 for axis in (0, 1)])
    displacement = rows[-1, 1:3] - rows[0, 1:3]
    truth_displacement = reference[-1] - reference[0]
    relative_error = np.linalg.norm((rows[:, 1:3] - rows[0, 1:3])
                                    - (reference - reference[0]), axis=1)
    return dict(interval_s=[float(rows[0, 0] - ORIGIN), float(rows[-1, 0] - ORIGIN)],
                samples=len(rows), displacement_m=displacement.tolist(),
                truth_displacement_m=truth_displacement.tolist(),
                net_displacement_error_m=float(np.linalg.norm(displacement - truth_displacement)),
                relative_position_error_max_m=float(np.max(relative_error)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("truth", type=Path)
    parser.add_argument("--trace", type=Path)
    args = parser.parse_args()
    source = Path(eskf.__file__).parent
    spec = importlib.util.spec_from_file_location("eskf_449_replay_node", source / "scripts/eskf_node.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    parameters = load_parameters(args.bag.parent / "eskf_localization.log",
                                 args.bag.parent / "sensor_preset.json")
    now = [0.0]
    outputs, recorded, recorded_diagnostics, trace = [], [], [], []
    rejection_windows = []
    slow_motion_zupts = 0
    gps_excluded = 0
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
        node.tf_broadcaster = None
        node.odom_pub = Mock()

        def capture(message):
            outputs.append([message.header.stamp.to_sec(), message.pose.pose.position.x,
                            message.pose.pose.position.y, message.twist.twist.linear.x])

        node.odom_pub.publish.side_effect = capture
        callbacks = {"/imu/data": node.imu_callback, "/gps": node.gps_callback,
                     "/Competition_topic": node.vehicle_state_callback,
                     "/odometry/pure": node.odometry_callback}
        with rosbag.Bag(str(args.bag)) as bag:
            for topic, message, receipt in bag.read_messages(
                    topics=list(callbacks) + ["/eskf/diagnostics", "/eskf/odom"],
                    end_time=rospy.Time.from_sec(ORIGIN + END)):
                now[0] = receipt.to_sec()
                relative = now[0] - ORIGIN
                if topic == "/eskf/diagnostics":
                    diagnostic = json.loads(message.data)
                    node.odometry_time_scale = diagnostic["odometry_time_scale"]
                    if 650 <= relative <= 710:
                        recorded_diagnostics.append(dict(t=relative, **diagnostic))
                    if START <= relative <= END:
                        trace.append(dict(t=relative, wheel=node.last_wheel_speed,
                                          velocity=node.filter.base_state()[1].tolist(),
                                          scale=node.odometry_time_scale,
                                          counters=dict(node.filter.counters),
                                          rejection_reasons=dict(getattr(node, "odometry_rejection_reasons", {})),
                                          motion=getattr(node, "_wheel_motion_confirmed", lambda _: False)(now[0])))
                elif topic == "/eskf/odom":
                    recorded.append([message.header.stamp.to_sec(), message.pose.pose.position.x,
                                     message.pose.pose.position.y, message.twist.twist.linear.x])
                elif topic == "/gps" and relative >= START:
                    gps_excluded += 1
                else:
                    before = node.filter.counters["zupt_accepted"]
                    reject_before = node.filter.counters["odometry_rejected"]
                    callbacks[topic](message)
                    if START <= relative <= END and topic == "/imu/data":
                        motion = getattr(node, "_wheel_motion_confirmed", lambda _: False)(message.header.stamp.to_sec())
                        if motion:
                            slow_motion_zupts += node.filter.counters["zupt_accepted"] - before
                    if 655 <= relative <= 705 and topic == "/odometry/pure":
                        rejection_windows.append(dict(
                            t=relative, delta=node.filter.counters["odometry_rejected"] - reject_before,
                            reason=getattr(node, "last_odometry_rejection_reason", None),
                            lag=node.last_predict_stamp.to_sec() - message.header.stamp.to_sec(),
                            anchor_age=(None if node.odometry_alignment.anchor_stamp is None
                                        else message.header.stamp.to_sec() - node.odometry_alignment.anchor_stamp)))
    # Scoring starts only after all sensor processing has finished.
    truth = load_truth(args.truth)
    selected = [min(recorded_diagnostics, key=lambda d: abs(d["t"] - t)) for t in (655, 705)]
    result = dict(source=str(source), sensor_replay=score(outputs, truth),
                  recorded=score(recorded, truth), gps_messages_excluded=gps_excluded,
                  zupts_during_confirmed_wheel_motion=slow_motion_zupts,
                  counters=dict(node.filter.counters),
                  rejection_reasons=dict(getattr(node, "odometry_rejection_reasons", {})),
                  recorded_655_705=[dict(t=d["t"], rejected=d["counters"]["odometry_rejected"],
                                         gps_age=d["gps_age_sec"],
                                         time_scale=d["odometry_time_scale"],
                                         last_nis=d["last_nis"].get("odometry_pose")) for d in selected],
                  replay_655_705=dict(callbacks=len(rejection_windows),
                                     rejected=sum(r["delta"] for r in rejection_windows),
                                     reasons=dict(Counter(r["reason"] for r in rejection_windows if r["delta"])),
                                     anchor_age_range_s=[min(r["anchor_age"] for r in rejection_windows),
                                                         max(r["anchor_age"] for r in rejection_windows)],
                                     max_abs_lag_s=max(abs(r["lag"]) for r in rejection_windows)))
    if args.trace:
        args.trace.write_text(json.dumps(dict(outputs=outputs, trace=trace,
                                             rejection_windows=rejection_windows)))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
