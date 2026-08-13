#!/usr/bin/env python3
"""Generate MORAI pedestrian scenarios at separated, recorded road poses."""

import copy
import json
import math
from pathlib import Path


HOME = Path("/home/bisa")
SCENARIO_DIR = HOME / "morai_man/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
SOURCE = SCENARIO_DIR / "far_pedestrian_v2_50k_20260802.json"
SOURCE_NETWORK = SCENARIO_DIR / "EgoNetwork/far_pedestrian_v2_50k_20260802_MN.json"
REFERENCE_PATH = HOME / "morai_man/reference_path.json"

# These poses are distributed over the 626 m road trajectory actually recorded
# from /Ego_topic.  Site 1 is the original scenario and is intentionally kept.
SITE_INDICES = {2: 390, 3: 754, 4: 1052, 5: 1335}
FAR_STANDOFF_METERS = 10.0
# Site 4's recorded centerline pose is on the inside guardrail after the car
# body settles.  Shift the entire pedestrian field and ego 5 m toward the
# drivable right-hand lane while keeping the same road segment and ranges.
SITE_RIGHT_OFFSETS_METERS = {4: 5.0, 5: 5.0}


def numeric_yaw(rot):
    return float(rot.get("yaw", 0.0))


def put_xyz(dst, x, y, z):
    dst.update({
        "x": float(x), "y": float(y), "z": float(z),
        "_x": f"{x:.3f}", "_y": f"{y:.3f}", "_z": f"{z:.3f}",
    })


def world_to_unity(dst, x, y, z):
    # MORAI scenario files store Unity positions as (-ENU.x, ENU.z, -ENU.y).
    put_xyz(dst, -x, z, -y)


def transform_xy(x, y, old_x, old_y, new_x, new_y, angle):
    dx, dy = x - old_x, y - old_y
    c, s = math.cos(angle), math.sin(angle)
    return new_x + c * dx - s * dy, new_y + s * dx + c * dy


def tangent_yaw(points, index, radius=5):
    lo, hi = max(0, index - radius), min(len(points) - 1, index + radius)
    return math.degrees(math.atan2(
        points[hi]["y"] - points[lo]["y"],
        points[hi]["x"] - points[lo]["x"],
    ))


def main():
    source = json.loads(SOURCE.read_text())
    network = json.loads(SOURCE_NETWORK.read_text())
    points = json.loads(REFERENCE_PATH.read_text())["waypoints"]
    old_pos = source["egoVehicle"]["initPosition"]["pos"]
    old_x, old_y, old_z = old_pos["x"], old_pos["y"], old_pos["z"]
    old_yaw = numeric_yaw(source["egoVehicle"]["initPosition"]["rot"])

    manifest = [{
        "site": 1, "scenario": SOURCE.stem,
        "x": old_x, "y": old_y, "z": old_z, "yaw": old_yaw,
    }]

    for site, index in SITE_INDICES.items():
        data = copy.deepcopy(source)
        target = points[index]
        anchor_x, anchor_y, new_z = target["x"], target["y"], old_z
        new_yaw = tangent_yaw(points, index)
        right_offset = SITE_RIGHT_OFFSETS_METERS.get(site, 0.0)
        anchor_x += right_offset * math.sin(math.radians(new_yaw))
        anchor_y -= right_offset * math.cos(math.radians(new_yaw))
        new_x = anchor_x - FAR_STANDOFF_METERS * math.cos(math.radians(new_yaw))
        new_y = anchor_y - FAR_STANDOFF_METERS * math.sin(math.radians(new_yaw))
        yaw_delta = new_yaw - old_yaw
        angle = math.radians(yaw_delta)

        ego_init = data["egoVehicle"]["initPosition"]
        put_xyz(ego_init["pos"], new_x, new_y, new_z)
        ego_init["rot"]["yaw"] = f"{new_yaw:.3f}"
        ego_init["vehicleSaveData"]["velocity"] = 0.0
        ego_init["vehicleSaveData"]["lastVelocity"] = {"x": 0.0, "y": 0.0, "z": 0.0}
        ego_init["vehicleSaveData"]["inputData"] = [0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0]

        for ped in data["pedestrianList"]:
            pos = ped["pos"]
            px, py = transform_xy(pos["x"], pos["y"], old_x, old_y, anchor_x, anchor_y, angle)
            pz = pos["z"] + (new_z - old_z)
            put_xyz(pos, px, py, pz)
            world_to_unity(ped["initPos"], px, py, pz)
            world_to_unity(ped["standardPos"], px, py, pz)
            ped["rot"]["yaw"] = f"{numeric_yaw(ped['rot']) + yaw_delta:.3f}"

        # Waypoint positions are in Unity coordinates, so convert to world,
        # apply the same rigid transform, and convert back.
        for path in data["waypointDataList"]:
            for waypoint in path.get("waypointData", []):
                pos = waypoint["pos"]
                wx, wy, wz = -pos["x"], -pos["z"], pos["y"]
                wx, wy = transform_xy(wx, wy, old_x, old_y, anchor_x, anchor_y, angle)
                wz += new_z - old_z
                world_to_unity(pos, wx, wy, wz)

        stem = f"far_pedestrian_v2_site{site}_20260802"
        output = SCENARIO_DIR / f"{stem}.json"
        output.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
        network_output = SCENARIO_DIR / "EgoNetwork" / f"{stem}_MN.json"
        network_output.write_text(json.dumps(network, ensure_ascii=False, indent=2) + "\n")
        manifest.append({
            "site": site, "scenario": stem,
            "x": new_x, "y": new_y, "z": new_z, "yaw": new_yaw,
            "path_index": index,
            "far_standoff_m": FAR_STANDOFF_METERS,
            "right_offset_m": right_offset,
        })

    manifest_path = HOME / "morai-mpc-agent-morai-lio-gps-integration/logs/far_v2/multisite_manifest.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    for item in manifest:
        print(
            f"site={item['site']} scenario={item['scenario']} "
            f"pose=({item['x']:.2f},{item['y']:.2f},{item['yaw']:.1f}deg)"
        )


if __name__ == "__main__":
    main()
