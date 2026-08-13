#!/usr/bin/env python3
"""Static and recorded-data gate for the standalone backup odometry."""

import argparse
import ast
import json
import os
import xml.etree.ElementTree as element_tree

import yaml


def attribute_chain(node):
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if isinstance(node, ast.Name):
        return ".".join([node.id] + list(reversed(parts)))
    return ""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--regression-json",
        default="logs/pure_odometry_offline_regression.json")
    args = parser.parse_args()

    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    workspace_src = os.path.dirname(package_dir)
    node_path = os.path.join(package_dir, "scripts", "pure_odometry_node.py")
    launch_path = os.path.join(package_dir, "launch", "odometry.launch")
    main_launch_path = os.path.join(
        workspace_src, "morai_launch", "launch", "morai.launch")
    eskf_config_path = os.path.join(
        workspace_src, "eskf", "config", "eskf_robust.yaml")
    regression_path = args.regression_json
    if not os.path.isabs(regression_path):
        regression_path = os.path.join(package_dir, regression_path)

    source = open(node_path, encoding="utf-8").read()
    tree = ast.parse(source, filename=node_path)
    callback = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
        and node.name == "vehicle_callback")
    accessed = sorted(set(
        attribute_chain(node) for node in ast.walk(callback)
        if isinstance(node, ast.Attribute)
        and attribute_chain(node).startswith("message.")))
    forbidden_fields = (
        "message.position", "message.heading", "message.acceleration",
        "message.velocity.y")
    violations = [
        field for field in forbidden_fields
        if any(value == field or value.startswith(field + ".")
               for value in accessed)]
    if violations:
        raise RuntimeError("forbidden Ego fields: {}".format(violations))
    required_fields = {"message.velocity.x", "message.wheel_angle"}
    if not required_fields.issubset(set(accessed)):
        raise RuntimeError("missing wheel-equivalent fields")
    for forbidden_type in ("GPSMessage", "PointCloud2"):
        if forbidden_type in source:
            raise RuntimeError("forbidden estimator type: " + forbidden_type)

    launch_root = element_tree.parse(launch_path).getroot()
    nodes = launch_root.findall("node")
    estimator_nodes = [
        node for node in nodes
        if node.attrib.get("name") == "pure_odometry"]
    if len(estimator_nodes) != 1:
        raise RuntimeError("odometry.launch must contain one estimator node")
    if estimator_nodes[0].attrib.get("type") != "pure_odometry_node.py":
        raise RuntimeError("unexpected estimator executable")
    launch_text = open(launch_path, encoding="utf-8").read()
    for forbidden in ("sensor_fusion", "eskf_node", "lio_sam", "/gps"):
        if forbidden in launch_text:
            raise RuntimeError("standalone launch dependency: " + forbidden)

    main_launch_text = open(main_launch_path, encoding="utf-8").read()
    if "start_pure_odometry" not in main_launch_text:
        raise RuntimeError("main MORAI launch must expose pure odometry")
    with open(eskf_config_path, encoding="utf-8") as stream:
        eskf_config = yaml.safe_load(stream)
    if bool(eskf_config.get("wheel_aiding_enabled", False)):
        raise RuntimeError("direct Ego wheel aiding must remain disabled")
    if not bool(eskf_config.get("odometry_aiding_enabled", False)):
        raise RuntimeError("validated odometry aiding must be enabled")
    if not bool(eskf_config.get("odometry_speed_aiding_enabled", False)):
        raise RuntimeError("validated odometry speed aiding must be enabled")

    with open(regression_path, encoding="utf-8") as stream:
        regression = json.load(stream)
    aggregate = regression.get("aggregate", {})
    if not aggregate.get("all_passed", False):
        raise RuntimeError("recorded-data regression did not pass")
    report = {
        "status": "PASS",
        "standalone_launch": launch_path,
        "estimator_vehicle_fields": sorted(required_fields),
        "gps_input": False,
        "lidar_input": False,
        "eskf_integrated": True,
        "offline_regression": aggregate,
    }
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
