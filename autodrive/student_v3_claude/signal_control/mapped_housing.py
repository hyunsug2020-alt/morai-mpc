"""Project the selected mapped signal into camera 4 to associate semantic boxes."""
import json
import math
from pathlib import Path

import numpy as np

from .camera import CAMERA_ID


class MappedSignalHousing:
    def __init__(self, lanes, sensor_file, light_file=None):
        self.lanes = lanes
        self.links = {link["idx"]: link for link in lanes.links}
        light_file = Path(light_file) if light_file else Path(__file__).resolve().parents[1] / "traffic_light_set.json"
        self.lights = {light["idx"]: light for light in json.loads(light_file.read_bytes())}
        sensor = json.loads(Path(sensor_file).read_bytes())
        camera = next(c for c in sensor["cameraList"] if int(c["m_SensorUniqueID"]) == CAMERA_ID)
        self.position = np.array([float(camera["pos"][k]) for k in ("x", "y", "z")])
        yaw, pitch, roll = (math.radians(float(camera["rot"][k])) for k in ("yaw", "pitch", "roll"))
        cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        self.camera_to_body = rz @ ry @ rx
        cc = camera["cc"]
        self.width, self.height = int(cc["cameraResWidth"]), int(cc["cameraResHeight"])
        self.focal = self.width / (2 * math.tan(math.radians(float(cc["cameraFOV"])) / 2))

    def validate(self, lines):
        for line in lines:
            light = self.lights.get(line["traffic_light_id"])
            if not light or light.get("type") != "car" or light.get("orientation") != "horizontal":
                raise ValueError("mapped traffic-light housing missing: " + line["traffic_light_id"])
            if line["link_id"] not in light.get("link_id_list", ()):
                raise ValueError("mapped signal and entry link disagree: " + line["traffic_light_id"])
            geometry = list(light.get("point", ())) + [light.get("heading"), light.get("width"), light.get("height")]
            if (len(geometry) != 6 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in geometry)
                    or light["width"] <= 0 or light["height"] <= 0):
                raise ValueError("invalid mapped traffic-light geometry")
            if light.get("sub_type") not in (["red", "yellow", "straight"], ["red", "yellow", "left", "straight"]):
                raise ValueError("unsupported mapped traffic-light lamp order")

    def ground_height(self, ego, line):
        link = self.links[line["link_id"]]
        if hasattr(self.lanes, "tree"):
            _, index = self.lanes.tree.query([ego["x"], ego["y"]])
            link = self.lanes.links[int(self.lanes.link_of[int(index)])]
        points = np.asarray(link["points"], dtype=float)
        at = int(np.argmin(np.sum((points[:, :2]-[ego["x"], ego["y"]])**2, axis=1)))
        return float(points[at, 2])

    def project(self, ego, line):
        light = self.lights[line["traffic_light_id"]]
        result = dict(traffic_light_id=light["idx"], layout="four" if "left" in light["sub_type"] else "three",
                      permission_policy=line.get("permission_policy", "green"),
                      window=None, expected_box=None)
        heading = math.radians(float(ego["heading_deg"]))
        ch, sh = math.cos(heading), math.sin(heading)
        body_to_world = np.array([[ch, -sh, 0], [sh, ch, 0], [0, 0, 1]])
        origin = np.array([float(ego["x"]), float(ego["y"]), self.ground_height(ego, line)])
        center = np.asarray(light["point"], dtype=float)
        angle = math.radians(float(light["heading"]))
        across = np.array([-math.sin(angle), math.cos(angle), 0.]) * float(light["width"]) / 2
        up = np.array([0., 0., float(light["height"]) / 2])
        corners = np.array([center + a*across + b*up for a in (-1, 1) for b in (-1, 1)])
        camera = ((corners-origin) @ body_to_world-self.position) @ self.camera_to_body
        if not np.isfinite(camera).all() or np.any(camera[:, 0] <= .1):
            return result
        u = self.width/2-self.focal*camera[:, 1]/camera[:, 0]
        v = self.height/2-self.focal*camera[:, 2]/camera[:, 0]
        expected = [float(u.min()), float(v.min()), float(u.max()), float(v.max())]
        # Account for pose/map housing offsets without including the road or hood.
        padding = max(8., .8*self.focal/float(camera[:, 0].min()))
        # Braking pitches the camera relative to the map's level-ground
        # projection. Recorded valid housings moved 12 pixels vertically;
        # keep horizontal association strict while budgeting two degrees
        # of pitch in the vertical search. Color still requires semantics.
        vertical_padding = padding + self.focal*math.tan(math.radians(2.))
        window = [max(0., expected[0]-padding), max(0., expected[1]-vertical_padding),
                  min(float(self.width), expected[2]+padding), min(float(self.height), expected[3]+vertical_padding)]
        result["expected_box"] = [expected[0]/self.width, expected[1]/self.height,
                                  expected[2]/self.width, expected[3]/self.height]
        if window[0] < window[2] and window[1] < window[3]:
            result["window"] = [window[0]/self.width, window[1]/self.height,
                                window[2]/self.width, window[3]/self.height]
        return result
