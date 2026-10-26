"""Camera roles shared by collection, training, inference, and the web UI."""
from __future__ import annotations

import json
from pathlib import Path

TRAFFIC_LIGHT_ID = 3
ALL_CLASS_IDS = (0, 1, 2, 3, 4, 5)
GENERAL_CLASS_IDS = (0, 1, 2, 4, 5)
CAMERA_GROUPS = {"general": ("front", "left", "right"), "signal": ("signal",), "all": ("front", "left", "right", "signal")}
BACKBONES = ("efficientvit_b1.r256_in1k", "efficientvit_b0.r224_in1k")
CAMERA_PAIRS = {
    "front": {"rgb_id": 1, "semantic_id": 4, "rgb_port": 12002, "semantic_port": 13002,
              "supervised_class_ids": list(GENERAL_CLASS_IDS), "student_rgb": True},
    "left": {"rgb_id": 2, "semantic_id": 8, "rgb_port": 12004, "semantic_port": 13004,
             "supervised_class_ids": list(GENERAL_CLASS_IDS), "student_rgb": True},
    "right": {"rgb_id": 3, "semantic_id": 9, "rgb_port": 12006, "semantic_port": 13006,
              "supervised_class_ids": list(GENERAL_CLASS_IDS), "student_rgb": True},
    "signal": {"rgb_id": 10, "semantic_id": 11, "rgb_port": 12008, "semantic_port": 12010,
               "supervised_class_ids": [TRAFFIC_LIGHT_ID], "student_rgb": True},
}


def supervised_class_ids(record):
    default = CAMERA_PAIRS.get(record.get("view"), {}).get("supervised_class_ids", ALL_CLASS_IDS)
    values = list(record.get("supervised_class_ids", default))
    if not values or len(set(values)) != len(values) or any(value not in ALL_CLASS_IDS for value in values):
        raise ValueError(f"invalid supervised classes: {values}")
    if record.get("view") == "signal" and values != [TRAFFIC_LIGHT_ID]:
        raise ValueError("signal camera must supervise only traffic_light (3)")
    if record.get("view") in ("front", "left", "right"):
        values = [value for value in values if value != TRAFFIC_LIGHT_ID]
    return values


def validate_sensor_preset(path: Path):
    cameras = {int(camera["m_SensorUniqueID"]): camera for camera in json.loads(path.read_text())["cameraList"]}
    for view, pair in CAMERA_PAIRS.items():
        rgb, semantic = (cameras[pair[key]] for key in ("rgb_id", "semantic_id"))
        for camera, kind, camera_type in ((rgb, "rgb", 0), (semantic, "semantic", 1)):
            cc = camera["cc"]
            if cc["cameraType"] != camera_type or cc["udpConfig"]["destinationPort"] != pair[kind + "_port"]:
                raise ValueError(f"{view}: {kind} type/port does not match {path}")
        if rgb["pos"] != semantic["pos"] or rgb["rot"] != semantic["rot"]:
            raise ValueError(f"{view}: RGB and semantic pose differs")
        for key in ("cameraResWidth", "cameraResHeight", "cameraFOV", "sensorPeriod"):
            if rgb["cc"][key] != semantic["cc"][key]:
                raise ValueError(f"{view}: RGB and semantic {key} differs")
