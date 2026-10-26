"""Closed-loop virtual drive of the real Student decision code on the competition map.

The real ``Runtime.infer`` (normal road, junction, highway and tunnel branches)
decides every frame. Only the sensors, the camera model and the vehicle are
simulated:

* ego: kinematic bicycle, pure pursuit on the published /avoid_waypoints,
  speed tracking the published cap with the follower's 3 m/s^2 rise/fall;
* objects: exact boxes, published as fused objects and LiDAR cars;
* LiDAR: box perimeters at 0.2 m, low objects invisible near the bumper;
* tunnel: 3-D wall and obstacle returns from the surveyed wall map;
* model: follows the route (the recorded model rarely proposed a usable
  avoidance), so every manoeuvre is the planners' own.

The clock advances by max(1/15 s, real decision time), so a slow decision costs
driving time as it does on the car. Usage:

    python virtual_sim.py            # every scenario
    python virtual_sim.py highway_cones intersection_3s
"""
from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import threading
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace

# Competition launch flags (start_*.sh) must be set before gui is imported.
for key, value in dict(STUDENT_FAST_TRAFFIC_GUARD="1", STUDENT_STOP_AT_STOP_LINE="1",
                       STUDENT_STRICT_STATIC_STOP="1", STUDENT_BYPASS_ALAT="3.0",
                       STUDENT_CLEARANCE="both").items():
    os.environ.setdefault(key, value)

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
# SIM_GUI_DIR runs another decision-code folder (e.g. an unpacked student_v4)
# against the same map, sensors and scenarios. Map data always comes from HERE.
GUI_DIR = Path(os.environ.get("SIM_GUI_DIR", str(HERE))).resolve()
sys.path.insert(0, str(HERE))
if GUI_DIR != HERE:
    sys.path.insert(0, str(GUI_DIR))

import gui  # noqa: E402
from gui import HdMapLanes, Runtime, RosPathPublisher, smooth_control_route  # noqa: E402
from lidar_corridor import LidarCorridor  # noqa: E402
from ego_body import body_corners as ego_body_corners  # noqa: E402
from traffic_planner import GapPlanner  # noqa: E402
try:
    from highway_planner import HighwayPlanner, NearFieldMemory  # noqa: E402
    from intersection_perception import build_intersection_regions  # noqa: E402
    from stop_line import COMPETITION_STOP_NODE_IDS, competition_stop_controller  # noqa: E402
except ImportError:          # older decision code without these sections
    HighwayPlanner = NearFieldMemory = build_intersection_regions = None
    COMPETITION_STOP_NODE_IDS = competition_stop_controller = None

DT = 1. / 15.
# SIM_CURVE_FOLLOWER=1: emulate the C++ path_follower curve speed (max |kappa| of the
# published path within 70 m ahead, kappa from 0.5 m central-difference headings,
# v <= sqrt(curve_alat / kappa); curve_alat 4.2, gui.CURVE_ALAT_ZONE_MPS2 inside
# gui.CURVE_ALAT_ZONE). Off by default: the other scenarios keep their behaviour.
CURVE_FOLLOWER = os.environ.get("SIM_CURVE_FOLLOWER", "0") == "1"
CURVE_ALAT_BASE = float(os.environ.get("SIM_CURVE_ALAT_BASE", "4.2"))
CURVE_LOOKAHEAD_M = 70.
DRIFT_GAIN = float(os.environ.get("SIM_DRIFT_GAIN", "0.3"))
HALF_L, HALF_W = 2.3175, .946
WHEELBASE = 3.0
TRUE_BODY_SHIFT = (3.845 - .79) / 2. # rear-axle reference: body centre 1.53 m ahead of it
LAUNCH_DEAD_S, LAUNCH_ACCEL_MPS2, LAUNCH_SLOW_MPS = .65, 2.1, 20. / 3.6
ACTIONS = ["KEEP_LANE", "TURN_LEFT", "TURN_RIGHT", "STOP", "YIELD", "LEFT_CHANGE", "RIGHT_CHANGE"]


# --------------------------------------------------------------------------- world
class Clock:
    def __init__(self, start=1_000_000.):
        self.now = start

    def monotonic(self):
        return self.now

    def time(self):
        return self.now


class Message:
    def __init__(self, data=None):
        self.data = data


class Sink:
    def __init__(self):
        self.last = None

    def publish(self, value):
        self.last = value


class Pose:
    def __init__(self):
        self.header = None
        self.pose = SimpleNamespace(position=SimpleNamespace(x=0., y=0.))


class PathMessage:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id=None)
        self.poses = []


def box_corners(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
        out.append((x + a * length / 2 * c - b * width / 2 * s, y + a * length / 2 * s + b * width / 2 * c))
    return np.asarray(out)


def box_gap(a, b):
    """Separation (m) of two convex boxes; <= 0 is contact."""
    best = -np.inf
    for poly in (a, b):
        for i in range(4):
            edge = poly[(i + 1) % 4] - poly[i]
            axis = np.array([-edge[1], edge[0]]) / max(1e-9, np.linalg.norm(edge))
            pa, pb = a @ axis, b @ axis
            best = max(best, pb.min() - pa.max(), pa.min() - pb.max())
    return float(best)


def npc_ego_body_headway(ego, actor):
    """Ego-body headway in the NPC's current forward corridor, or None.

    The ego pose is its rear axle. Project its full oriented body onto the
    NPC tangent instead of treating that pose as a symmetric body centre.
    This is a reacting-NPC fixture model, not a forecast of native NPC policy.
    """
    values = (ego.x, ego.y, ego.yaw, *actor.xy, actor.yaw, actor.length, actor.width)
    if not np.isfinite(values).all() or actor.length <= 0. or actor.width <= 0.:
        raise ValueError("NPC body reaction requires finite poses and positive dimensions")
    # Use the canonical rear-axle footprint directly. Test collision wrappers
    # may replace box_corners, which must not shift this body a second time.
    corners = ego_body_corners(ego.x, ego.y, ego.yaw)
    delta = corners - actor.xy
    c, s = math.cos(actor.yaw), math.sin(actor.yaw)
    along = delta[:, 0] * c + delta[:, 1] * s
    lateral = -delta[:, 0] * s + delta[:, 1] * c
    if (along.max() <= 0. or along.min() >= 60.
            or lateral.min() >= actor.width / 2.
            or lateral.max() <= -actor.width / 2.):
        return None
    return float(along.min() - actor.length / 2.)


class Actor:
    """A box; moving actors follow a world polyline at constant speed."""
    def __init__(self, name, kind, xy, yaw_deg, length, width, height=1.5,
                 speed=0., path=None, uid=None, start_time=0.):
        self.name, self.kind = name, kind
        self.xy = np.asarray(xy, float)
        self.yaw = math.radians(yaw_deg)
        self.length, self.width, self.height = length, width, height
        self.speed, self.uid = speed, uid
        self.path = None if path is None else np.asarray(path, float)
        self.start_time = start_time
        self.velocity = np.zeros(2)
        if self.path is not None:
            self.arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.path, axis=0), axis=1))]
            self.s = 0.
            self._place()

    def _place(self):
        s = min(self.s, self.arc[-1])
        x = np.interp(s, self.arc, self.path[:, 0]); y = np.interp(s, self.arc, self.path[:, 1])
        x2 = np.interp(min(s + .5, self.arc[-1]), self.arc, self.path[:, 0])
        y2 = np.interp(min(s + .5, self.arc[-1]), self.arc, self.path[:, 1])
        if s < self.arc[-1] - .1:
            self.yaw = math.atan2(y2 - y, x2 - x)
        self.xy = np.array([x, y])

    def step(self, t, dt):
        if self.path is None:
            return
        moving = t >= self.start_time and self.s < self.arc[-1]
        old = self.xy.copy()
        if moving:
            self.s += self.speed * dt
            self._place()
        self.velocity = (self.xy - old) / dt

    @property
    def active(self):
        # persist: a person who walked to a spot and stays there (auto run 268).
        return self.path is None or self.s < self.arc[-1] - .05 or getattr(self, "persist", False)

    def corners(self):
        return box_corners(self.xy[0], self.xy[1], self.yaw, self.length, self.width)

    def as_object(self):
        speed = float(np.linalg.norm(self.velocity))
        name = "lidar_car" if self.kind == "npc" else self.name
        return (float(self.xy[0]), float(self.xy[1]), max(self.length, self.width), name, self.kind,
                speed, self.length, self.width, math.degrees(self.yaw),
                float(self.velocity[0]), float(self.velocity[1]), self.uid or id(self) % 100000,
                0., None, True)

    def sensed(self, t, ego_xy, ego_yaw=0.):
        ped = getattr(self, "ped_noise", None)
        if ped is not None:
            return self.ped_sensed(t, ego_xy, ego_yaw, ped)
        return self.vehicle_sensed(t, ego_xy)

    def ped_sensed(self, t, ego_xy, ego_yaw, cfg):
        """Fused camera/LiDAR pedestrian as logged in auto runs 243/247: first seen only
        ``range_m`` away, frames dropped (``drop_p``), position jitter (``pos_sigma``)
        and a fused velocity that reads 0 (heading = ego heading) on ``v_zero_p`` of
        the frames although the person walks at 0.45 m/s."""
        rng = cfg.setdefault("rng", np.random.default_rng(int(self.uid or 1)))
        if float(np.hypot(*(self.xy - ego_xy))) > cfg.get("range_m", 1e9):
            return None
        if rng.random() < cfg.get("drop_p", 0.):
            return None
        obj = list(self.as_object())
        sigma = cfg.get("pos_sigma", 0.)
        if sigma:
            obj[0] += float(rng.normal(0., sigma)); obj[1] += float(rng.normal(0., sigma))
        if rng.random() < cfg.get("v_zero_p", 0.):
            obj[5], obj[9], obj[10], obj[8] = 0., 0., 0., math.degrees(ego_yaw)
        return tuple(obj)

    def vehicle_sensed(self, t, ego_xy):
        """The LiDAR vehicle tracker's view of this actor, or None while it is lost.

        Without ``noise`` the track is perfect. With it (live runs 200/210/211/222 at
        the roundabout): a new track has no valid velocity for ``vok_delay_s``
        (v_ok False, velocity 0, as lidar_vehicle_detection reports it); the first
        ``drops`` times the car comes within ``drop_within_m`` of the ego it vanishes
        for ``drop_s`` and returns under a new id (0.9-2.6 s blackouts measured)."""
        obj = self.as_object()
        noise = getattr(self, "noise", None)
        if noise is None:
            return obj
        if noise.get("first_seen") is None:
            noise["first_seen"] = t
            noise["vok_until"] = t + noise.get("initial_vok_s", noise["vok_delay_s"])
        distance = float(np.hypot(*(self.xy - ego_xy)))
        if (noise["drops_left"] > 0 and distance <= noise["drop_within_m"]
                and t >= noise.get("drop_after_s", 0.) and not noise.get("dropping")):
            noise["drops_left"] -= 1
            noise["drop_until"] = t + noise["drop_s"]
            noise["dropping"] = True
        if noise.get("dropping"):
            if t < noise["drop_until"]:
                return None
            noise["dropping"] = False
            self.uid = (self.uid or 0) + 1000          # re-identified as a new track
            noise["vok_until"] = t + noise["vok_delay_s"]
            noise.setdefault("events", []).append(round(t, 2))
        if t < noise["vok_until"]:
            obj = obj[:5] + (0.,) + obj[6:9] + (0., 0., self.uid) + obj[12:14] + (False,)
        else:
            obj = obj[:11] + (self.uid,) + obj[12:]
        return obj

    def perimeter(self, spacing=.2):
        c = self.corners()
        pts = []
        for i in range(4):
            a, b = c[i], c[(i + 1) % 4]
            n = max(2, int(np.linalg.norm(b - a) / spacing))
            pts.append(a + (b - a) * np.linspace(0., 1., n, endpoint=False)[:, None])
        return np.vstack(pts)


class QueueCar(Actor):
    """A car on a path that brakes (decel_mps2) to stand with its centre at arc stop_s,
    stands until resume_t (None: for good), then drives on at resume_kmh. Run 264: a
    ~35 km/h car ahead slowing into the queue 10 m before the first signal junction."""
    def __init__(self, name, path, kmh, stop_s, *, decel_mps2=2.5, resume_t=None, resume_kmh=25.,
                 uid=None, length=4.6, width=1.9):
        super().__init__(name, "npc", path[0], 0., length, width, 1.5, speed=kmh / 3.6, path=path, uid=uid)
        self.cruise_mps, self.stop_s, self.decel = kmh / 3.6, float(stop_s), decel_mps2
        self.resume_t, self.resume_mps, self.resumed = resume_t, resume_kmh / 3.6, False

    def step(self, t, dt):
        old = self.xy.copy()
        if not self.resumed and self.resume_t is not None and t >= self.resume_t and self.s >= self.stop_s - .05:
            self.resumed = True
        if self.resumed:
            self.speed = min(self.resume_mps, self.speed + 1.5 * dt)
        else:
            room = max(0., self.stop_s - self.s)
            self.speed = min(self.cruise_mps, math.sqrt(2. * self.decel * room))
        self.s = min(self.s + self.speed * dt, self.arc[-1] if self.resumed else self.stop_s)
        self._place()
        self.velocity = (self.xy - old) / dt


# --------------------------------------------------------------------------- runtime
class SimulatedVehicleReceiver(gui.VehicleReceiver):
    """Feed synthetic messages through the production receiver contract.

    Only the ROS subscription is absent. Snapshot conversion, source identity,
    receipt aging and atomic callback replacement use the real implementation.
    Disabling delivery models a lost topic without refreshing its last frame.
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.record, self.received = {}, 0.
        self.delivery_enabled = True

    def publish_rows(self, rows, *, stamp, ready=True, active=True, **metadata):
        if not self.delivery_enabled:
            return
        vehicles = [dict(position=[row[0], row[1]], velocity=[row[9], row[10]],
                         size=[row[6], row[7], 1.5], heading_deg=row[8], uid=row[11],
                         velocity_valid=bool(row[14]),
                         prediction_age_s=float(row[15]) if len(row) > 15 else 0.,
                         stamp=float(stamp)) for row in rows]
        record = dict(metadata, stamp=float(stamp), ready=bool(ready), active=bool(active),
                      vehicles=vehicles, hz=15.)
        self._on_message(Message(json.dumps(record)))

    def close(self):
        pass  # There is no ROS subscription in the synthetic runtime.


class SimulatedRuntime:
    def __init__(self, waypoints, smooth, lanes, *, highway=True, stop_line=True, tunnel=True,
                 zone_kmh=float(os.environ.get("SIM_ZONE_KMH", "34.73")), model_delay_s=0., model_offset_m=0., model_points=None):
        r = Runtime.__new__(Runtime)
        self.r = r
        self.clock = Clock()
        r.args = SimpleNamespace(force_student_avoidance=True, max_route_distance_m=100.,
                                 cruise_kmh=90., status=Path(tempfile.gettempdir()) / "sim_status.json")
        r.device = torch.device("cpu")
        r.config = dict(frames=1, ego_steps=4, route_points=20, width=64, height=32)
        r.actions = ACTIONS
        r.gallery, r.gallery_matrix = [], np.empty((0, 1))
        r.receiver = SimpleNamespace(snapshot=lambda count: [], rates=lambda: {}, stale_views=lambda: [])
        r.image_tensor = lambda rows: (torch.zeros(1), None)
        r.ego_tensor = lambda current: torch.zeros(1)
        r.model_camera_fresh = lambda max_age_s=.5: True
        r.model = self.model
        r.input_error = None
        r.clearance = "both"
        r.lanes = lanes
        r.waypoints = [dict(x=float(x), y=float(y)) for x, y in waypoints]
        r.waypoint_xy = np.asarray(waypoints, float)
        r.control_waypoint_xy = smooth
        r.route_cumulative = np.r_[0., np.cumsum(np.linalg.norm(np.diff(r.waypoint_xy, axis=0), axis=1))]
        r.speed_zones = SimpleNamespace(kmh_at=lambda index: (zone_kmh, {"kmh": zone_kmh}))
        r.zone_kmh, r.zone = zone_kmh, None
        r.gap_planner = GapPlanner(lanes.links)
        from junction_recovery import JunctionRecovery
        r.junction_recovery = JunctionRecovery()
        r.stop_line_controller = competition_stop_controller(lanes.stop_lines) if stop_line else None
        r.intersection_regions = (build_intersection_regions(
            lanes.stop_lines, lanes.route_junction, lanes.route_station, COMPETITION_STOP_NODE_IDS)
            if stop_line else [])
        r._intersection_vehicle_status = None
        r.highway = HighwayPlanner(HERE / "highway_mission.json", r.waypoint_xy) if highway else None
        if NearFieldMemory is not None:
            r.near_field_memory = NearFieldMemory(retain_measured_points=True) \
                if "retain_measured_points" in NearFieldMemory.__init__.__code__.co_varnames else NearFieldMemory()
        r.lidar = LidarCorridor(None, x_range=(-75., 100.))
        r.latencies = deque(maxlen=100)
        r.avoidance_direction, r.avoidance_latch_until = 0., 0.
        r.last_result, r.last_hazard = None, None
        r.ego_history, r.latest_status = [], {}
        r._decision_log = SimpleNamespace(write=lambda line: None, dropped=0, close=lambda: None)
        self.objects = []
        self.cars = []
        r.object_receiver = SimpleNamespace(age_s=.01, received=0., src="sim")
        r.object_receiver.snapshot = lambda max_age_s=1.: list(self.objects)
        # vehicle_ready False: the detector topic is not ready; like the real
        # VehicleReceiver.snapshot it then lists no cars at all.
        self.vehicle_ready = True
        r.vehicle_receiver = SimulatedVehicleReceiver()
        self.ego = None
        r.ego_receiver = SimpleNamespace(snapshot=lambda count: (self.ego, [self.ego] * count, 15.),
                                         at_stamp=lambda stamp: dict(self.ego, stamp=stamp))
        publisher = RosPathPublisher.__new__(RosPathPublisher)
        publisher.Float32 = publisher.String = Message
        publisher.PoseStamped, publisher.PathMessage = Pose, PathMessage
        publisher.rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda: 1.))
        for name in ("waypoints_pub", "velocity_pub", "mode_pub", "path_pub", "status_pub"):
            setattr(publisher, name, Sink())
        publisher.cruise_mps = 90. / 3.6
        publisher.publish_corridor = lambda *args: None
        publisher.publish_camera_rates = lambda *args: None
        publisher.publish_tunnel_constraint = lambda *args, **kwargs: None
        r.path_publisher = publisher
        self.tunnel_scan = None
        if tunnel:
            from tunnel_map import TunnelMap
            from tunnel_no_go import TunnelNoGoPublisher, TunnelNoGoReceiver
            r.tunnel = TunnelMap.load(HERE / "tunnel_wall_map.json")
            r._tunnel_cache = r._tunnel_plan_saved = r._tunnel_wall_anchor = None
            r.tunnel_odometry = SimpleNamespace(at_stamp=lambda stamp: dict(self.ego, stamp=stamp))
            r.tunnel_no_go_publisher = TunnelNoGoPublisher(topic=None)
            r.tunnel_no_go_receiver = TunnelNoGoReceiver(topic=None, clock=lambda: self.clock.now,
                                                         source_clock=lambda: self.clock.now)
            r.lidar.scan_snapshot = lambda: self.tunnel_scan
            if hasattr(r, "init_tunnel_model_policy"):
                r.init_tunnel_model_policy()   # STUDENT_TUNNEL_POLICY=model route scope / lane hold; no-op otherwise
        else:
            r.tunnel = None
        self.model_delay_s = model_delay_s
        self.model_offset_m = model_offset_m
        # model_points: truncate the route-copy model path (curve scenarios: the 9.5 m
        # route copy won the lattice at 58 km/h past idx 1650, shorter than the pure-
        # pursuit look-ahead, and the sim car ran off; the real model never won there).
        self.model_points = model_points

    def model(self, images, ego_input, route_input):
        """Route-following model: KEEP_LANE, path = the model route."""
        route = route_input[0, :, :2].float().clone()
        logits = torch.full((1, len(ACTIONS)), -8.)
        logits[0, 0] = 8.
        if self.model_offset_m:
            # A model that swerves with nothing there (as in the 2026-10-03 logs).
            route[:, 1] += self.model_offset_m * torch.clamp(route[:, 0] / 8., 0., 1.)
            logits[0, 0], logits[0, ACTIONS.index("LEFT_CHANGE")] = -8., 8.
        return dict(action_logits=logits, concept_logits=torch.zeros(1, 4),
                    scene=torch.zeros(1, 8), path=(route[:self.model_points] if self.model_points else route)[None])


# --------------------------------------------------------------------------- sim
class Sim:
    def __init__(self, world, start_idx, *, speed_kmh=0., actors=(), end_idx=None,
                 duration_s=120., tunnel_cones=(), lateral_offset=0., start_pose=None,
                 exit_open=False, vfov=False, drift=False, decision_s=None, match_fail=(),
                 vehicle_not_ready=(), launch_lag=False, heading_bias=None, steer_tau_s=None,
                 true_body=None, rtf=None, npc_reaction_geometry="legacy_point",
                 wall_shift_m=0., **runtime_options):
        if npc_reaction_geometry not in ("legacy_point", "ego_body"):
            raise ValueError("npc_reaction_geometry must be 'legacy_point' or 'ego_body'")
        # Preserve recorded adversarial fixtures by default. Corrected reacting
        # NPC geometry must be selected explicitly and is recorded in results.
        self.npc_reaction_geometry = npc_reaction_geometry
        self.world = world
        self.true_body = (os.environ.get("SIM_TRUE_BODY") == "1") if true_body is None else bool(true_body)
        # rtf: MORAI's real-time factor on the loaded PC (0.55-0.63 in auto runs 243/247).
        # Each decision (decision_s of wall clock) moves the world decision_s * rtf of
        # simulator time; the planner's clock (wall) runs 1/rtf faster.
        self.rtf = rtf
        # steer_tau_s: first-order lag of the steering (the MPC trails a newly published
        # path by ~0.3-0.5 s; the default follower turns the wheel at 400 deg/s).
        self.steer_tau_s = steer_tau_s
        # wall_shift_m: true tunnel walls this far left (+) of the surveyed wall map. The
        # wall match then puts the car -wall_shift_m off truth while the route keeps the
        # true frame (run 271: matched pose 0.5-0.8 m left of MORAI truth all tunnel long).
        self.wall_shift_m = float(wall_shift_m)
        # vehicle_not_ready: (t0, t1) windows in which the LiDAR vehicle detector is not ready.
        self.vehicle_not_ready = tuple(vehicle_not_ready)
        # launch_lag: measured standstill launch of the real car (dead time + 2.1 m/s^2).
        self.launch_lag, self.launch_wait = bool(launch_lag), 0.
        # heading_bias(route_idx) -> deg added to the localized heading the planner and the
        # follower see (run 227: the ESKF heading was 2-5 deg clockwise of MORAI truth in the
        # right bend, idx 1340-1540). LiDAR stays sensor-true, so far returns of the left
        # guardrail land in lane 2 and the car drifts left of every re-anchored path.
        self.heading_bias = heading_bias
        self.yaw_bias = 0.
        self.frame = 0
        # drift: emulate the MPC's measured tracking error toward the guardrail in the
        # S-curve (racing_line.json drift_obs_m at drift_v_kmh, scaled with speed^2):
        # the vehicle tracks the published path shifted left by it.
        self.drift = None
        if drift:
            payload = json.loads((HERE / "racing_line.json").read_text())
            table = np.zeros((len(world["smooth"]), 2)); table[:, 1] = 1.
            first = int(payload["first_idx"])
            table[first:first + len(payload["drift_obs_m"]), 0] = payload["drift_obs_m"]
            table[first:first + len(payload["drift_v_kmh"]), 1] = payload["drift_v_kmh"]
            self.drift = table
        self.rt = SimulatedRuntime(world["waypoints"], world["smooth"], world["lanes"], **runtime_options)
        wp = world["smooth"]
        heading = math.atan2(*(wp[start_idx + 3] - wp[start_idx])[::-1])
        normal = np.array([-math.sin(heading), math.cos(heading)])
        self.x, self.y = (wp[start_idx] + normal * lateral_offset)
        if start_pose is not None:          # (x, y, heading_deg): e.g. arriving skewed
            self.x, self.y, heading = float(start_pose[0]), float(start_pose[1]), math.radians(start_pose[2])
        # exit_open: no ground returns beyond the tunnel exit (open sky outside),
        # so the LiDAR coverage toward the exit reads short.
        self.exit_open = exit_open
        # vfov: VLP16 vertical field of view (lowest beam -15 deg, laser 1.75 m above
        # the road). A return at height z is visible only beyond (1.75-z)/tan 15 deg
        # from the laser: a 0.8 m barrier vanishes inside ~3.6 m (run 196).
        self.vfov = vfov
        self.yaw, self.v, self.steer = heading, speed_kmh / 3.6, 0.
        self.actors = list(actors)
        self.end_idx = end_idx
        self.duration = duration_s
        self.tunnel_map = getattr(self.rt.r, "tunnel", None)
        # decision_s: at most one decision per decision_s (a loaded PC: run 222 decided
        # at 3-7 Hz, 1.2-1.8 Hz when stopped). match_fail: (t0, t1) windows of sim time in
        # which the tunnel wall match fails, forcing the odometry fallback.
        self.decision_s = decision_s
        self.match_fail = tuple(match_fail)
        self._t = -1.
        if self.match_fail and self.tunnel_map is not None:
            real_process = self.tunnel_map.process
            def process(xyz, ego, plane, _real=real_process):
                if any(a <= self._t < b for a, b in self.match_fail):
                    return dict(valid=False, reason="forced wall match failure (sim)", aligned_ego=dict(ego),
                                obstacle_points=np.empty((0, 2)), wall_stats={}, constraint=None)
                return _real(xyz, ego, plane)
            self.tunnel_map.process = process
        self.log = []

    # sensors ----------------------------------------------------------------
    def local(self, pts, bias=0.):
        c, s = math.cos(self.yaw + bias), math.sin(self.yaw + bias)
        d = np.asarray(pts, float) - [self.x, self.y]
        return np.column_stack((d[:, 0] * c + d[:, 1] * s, -d[:, 0] * s + d[:, 1] * c))

    def lidar_points(self):
        pts = []
        for actor in self.actors:
            if not actor.active or getattr(actor, "ghost", None) is not None:
                continue
            local = self.local(actor.perimeter())
            rng = np.hypot(local[:, 0] - 1.05, local[:, 1])
            # lidar_range: farthest VLP16 returns of this actor (run 264: the car ahead first
            # showed at 30 m, one ring, after the ground filter).
            keep = (rng < getattr(actor, "lidar_range", 70.)) & (rng > (3.3 if actor.height < .8 else 1.2))
            pts.append(local[keep])
        pts = np.vstack(pts) if pts else np.zeros((0, 2))
        if len(pts):
            pts = np.unique(np.round(pts / .2).astype(np.int32), axis=0).astype(np.float32) * .2
        return pts.astype(np.float32)

    def tunnel_scan(self, stamp):
        tm = self.tunnel_map
        if tm is None:
            return None
        walls = []
        ego_xy = np.array([self.x, self.y])
        along = (ego_xy - tm.origin) @ tm.axis
        for offset in (tm.left + self.wall_shift_m, tm.right + self.wall_shift_m):
            s = along + np.arange(-30., 60., .25)
            s = s[(s >= tm.start - 5.) & (s <= tm.end + 5.)]
            base = tm.origin + s[:, None] * tm.axis + offset * tm.normal
            for z in (.4, .9, 1.4, 1.9):
                walls.append(np.column_stack((self.local(base), np.full(len(base), z))))
        for actor in self.actors:
            if actor.active:
                local = self.local(actor.perimeter())
                rng = np.hypot(local[:, 0] - 1.05, local[:, 1])
                if self.vfov:
                    for z in sorted(set(np.arange(.35, actor.height, .3).tolist()) | {actor.height - .05}):
                        seen = local[(rng < 70.) & (rng > (1.75 - z) / math.tan(math.radians(15.)))]
                        walls.append(np.column_stack((seen, np.full(len(seen), z))))
                    continue
                local = local[(rng < 70.) & (rng > (3.3 if actor.height < .8 else 1.2))]
                for z in np.arange(.35, actor.height, .3):
                    walls.append(np.column_stack((local, np.full(len(local), z))))
        az = np.radians(np.arange(-180., 180., .5))
        ring = np.column_stack((30 * np.cos(az), 30 * np.sin(az), np.zeros(len(az))))
        if self.exit_open:
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            world = np.column_stack((self.x + ring[:, 0] * c - ring[:, 1] * s,
                                     self.y + ring[:, 0] * s + ring[:, 1] * c))
            ring = ring[(world - tm.origin) @ tm.axis <= tm.end]
        walls.append(ring)
        xyz = np.vstack(walls).astype(np.float32)
        xyz.setflags(write=False)
        plane = np.zeros(3); plane.setflags(write=False)
        return dict(xyz=xyz, ground_plane=plane, stamp=stamp, sequence=int(stamp * 15), age_s=.01)

    # vehicle ----------------------------------------------------------------
    def drifted(self, waypoints, r):
        """Published path shifted left by the measured MPC drift (Sim(drift=True))."""
        if self.drift is None or len(waypoints) < 3 or waypoints[0].get("gear") == "R":
            return waypoints
        idx = int((r.latest_status.get("path") or {}).get("nearest_idx", 0))
        drift, v_ref = self.drift[min(max(idx, 0), len(self.drift) - 1)]
        # The measured drift is closed loop (the planner re-anchors at the car); the
        # injected path error is SIM_DRIFT_GAIN of it (calibrated on run 203, lane centre).
        e = min(2., DRIFT_GAIN * float(drift) * (self.v * 3.6 / max(float(v_ref), 1.)) ** 2)
        self.drift_now = e
        if e < 1e-3:
            return waypoints
        p = np.array([[w["x"], w["y"]] for w in waypoints], float)
        t = np.gradient(p, axis=0); t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
        q = p + e * np.column_stack((-t[:, 1], t[:, 0]))
        return [dict(w, x=float(a), y=float(b)) for w, (a, b) in zip(waypoints, q)]

    def follower_curve_cap(self, waypoints, route_idx):
        """C++ path_follower curve speed on the published path (SIM_CURVE_FOLLOWER)."""
        if len(waypoints) < 5:
            return None
        p = np.array([[w["x"], w["y"]] for w in waypoints], float)
        h = np.r_[math.atan2(*(p[1] - p[0])[::-1]), np.arctan2(p[2:, 1] - p[:-2, 1], p[2:, 0] - p[:-2, 0]),
                  math.atan2(*(p[-1] - p[-2])[::-1])]
        k = np.zeros(len(p))
        dh = (h[2:] - h[:-2] + np.pi) % (2 * np.pi) - np.pi
        k[1:-1] = dh / np.maximum(np.linalg.norm(p[2:] - p[:-2], axis=1), 1e-6)
        near = int(np.argmin(np.hypot(p[:, 0] - self.x, p[:, 1] - self.y)))
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
        window = (arc >= arc[near]) & (arc <= arc[near] + CURVE_LOOKAHEAD_M)
        kmax = float(np.max(np.abs(k[window]))) if window.any() else 0.
        zone = getattr(gui, "CURVE_ALAT_ZONE", (0, -1))
        alat = (max(CURVE_ALAT_BASE, getattr(gui, "CURVE_ALAT_ZONE_MPS2", CURVE_ALAT_BASE))
                if zone[0] <= route_idx <= zone[1] else CURVE_ALAT_BASE)
        self.curve_kmax = kmax
        return max(10. / 3.6, math.sqrt(alat / kmax)) if kmax > 1e-3 else None

    def control(self, waypoints, target):
        # Reverse escape publishes gear "R" waypoints behind the car: the sim used to
        # drive them forward, so every reverse in a sim was really a forward creep.
        reverse = bool(waypoints) and waypoints[0].get("gear") == "R"
        if reverse != getattr(self, "reversing", False):
            if self.v > .05:                       # change gear only at standstill
                self.v = max(0., self.v - 5. * DT)
                self.x += self.v * math.cos(self.yaw) * DT * (-1. if getattr(self, "reversing", False) else 1.)
                self.y += self.v * math.sin(self.yaw) * DT * (-1. if getattr(self, "reversing", False) else 1.)
                return
            self.reversing = reverse
        if reverse:
            pts = np.array([[w["x"], w["y"]] for w in waypoints])
            local = self.local(pts)
            behind = np.flatnonzero((local[:, 0] < -1.) & (np.hypot(local[:, 0], local[:, 1]) >= 3.))
            desired = 0.
            if len(behind):
                gx, gy = local[behind[0]]
                desired = -math.atan(WHEELBASE * 2. * gy / max(1e-3, gx * gx + gy * gy))
                desired = max(-math.radians(35), min(math.radians(35), desired))
            rate = math.radians(400.) * DT
            self.steer += max(-rate, min(rate, desired - self.steer))
            accel = max(-5., min(2., (target - self.v) / .25))
            self.v = max(0., self.v + accel * DT)
            self.x -= self.v * math.cos(self.yaw) * DT
            self.y -= self.v * math.sin(self.yaw) * DT
            self.yaw -= self.v / WHEELBASE * math.tan(self.steer) * DT
            return
        if waypoints:
            pts = np.array([[w["x"], w["y"]] for w in waypoints])
            local = self.local(pts, self.yaw_bias)
            look = max(4., .55 * self.v + 2.)
            ahead = np.flatnonzero((local[:, 0] > 0.) & (np.hypot(local[:, 0], local[:, 1]) >= look))
            if len(ahead):
                gx, gy = local[ahead[0]]
                curvature = 2. * gy / max(1e-3, gx * gx + gy * gy)
                desired = math.atan(WHEELBASE * curvature)
                desired = max(-math.radians(35), min(math.radians(35), desired))
                rate = math.radians(400.) * DT
                if self.steer_tau_s:
                    desired = self.steer + (desired - self.steer) * min(1., DT / float(self.steer_tau_s))
                self.steer += max(-rate, min(rate, desired - self.steer))
        accel_max = 2.9
        if self.launch_lag:
            # Measured MORAI launch (runs 200/210/211/222, junction): 5 km/h after
            # 1.3 s, 10 km/h after 1.95 s from the first non-zero command, i.e. ~0.65 s
            # dead time, then ~2.1 m/s^2. The default model reached 10 km/h in 0.9 s.
            if self.v < .05:
                self.launch_wait = self.launch_wait + DT if target > .05 else 0.
                if self.launch_wait < LAUNCH_DEAD_S:
                    target = 0.
            else:
                self.launch_wait = 0.
            if self.v < LAUNCH_SLOW_MPS:
                accel_max = LAUNCH_ACCEL_MPS2
        accel = (target - self.v) / .25
        accel = max(-3.5 if target > .05 else -5., min(accel_max, accel))
        self.v = max(0., self.v + accel * DT)
        self.x += self.v * math.cos(self.yaw) * DT
        self.y += self.v * math.sin(self.yaw) * DT
        self.yaw += self.v / WHEELBASE * math.tan(self.steer) * DT

    def npc_follow_ego(self, actor):
        """Lane NPCs brake for the ego car ahead in their lane (MORAI NPC traffic keeps
        a headway); without this every slowdown in front of them was a rear-end."""
        cruise = getattr(actor, "cruise", actor.speed)
        actor.cruise = cruise
        want = cruise
        geometry = getattr(self, "npc_reaction_geometry", "legacy_point")
        if geometry == "legacy_point":
            c, s = math.cos(actor.yaw), math.sin(actor.yaw)
            d = np.array([self.x, self.y]) - actor.xy
            lon, lat = d[0] * c + d[1] * s, -d[0] * s + d[1] * c
            if 0. < lon < 60. and abs(lat) < 2.3:
                gap = lon - HALF_L - actor.length / 2
                want = min(cruise, max(0., self.v + .5 * (gap - (5. + 1.2 * actor.speed))))
        elif geometry == "ego_body":
            gap = npc_ego_body_headway(self, actor)
            if gap is not None:
                want = min(cruise, max(0., self.v + .5 * (gap - (5. + 1.2 * actor.speed))))
        else:
            raise ValueError("npc_reaction_geometry must be 'legacy_point' or 'ego_body'")
        actor.speed = float(np.clip(want, actor.speed - 4. * DT, actor.speed + 1.5 * DT))

    # loop -------------------------------------------------------------------
    def run(self):
        rt, r = self.rt, self.rt.r
        clock = rt.clock
        real_monotonic, real_time = time.monotonic, time.time
        time.monotonic, time.time = clock.monotonic, clock.time
        stats = dict(collision=None, min_gap=np.inf, stop_s=0., longest_stop_s=0., t=0.,
                     reached=False, distance=0., stop_reasons={}, modes={}, speeds=[],
                     outside_max_m=0., outside_sum_m=0., outside_samples=0,
                     npc_reaction_geometry=getattr(self, "npc_reaction_geometry", "legacy_point"))
        current_stop = 0.
        try:
            start = clock.now
            while clock.now - start < self.duration:
                t = clock.now - start
                self._t = t
                for actor in self.actors:
                    if getattr(actor, "cruise_profile", None) is not None:
                        actor.cruise = actor.cruise_profile(t)
                    if getattr(actor, "follow_ego", False):
                        self.npc_follow_ego(actor)
                    trigger = getattr(actor, "start_within_m", None)
                    if trigger is not None and not getattr(actor, "triggered", False):
                        # MORAI-style trigger: the person starts walking once we are this close.
                        actor.start_time = 1e9
                        if float(np.hypot(*(actor.xy - [self.x, self.y]))) <= trigger:
                            actor.triggered, actor.start_time = True, t
                    actor.step(t, DT)
                idx_now = int((r.latest_status.get("path") or {}).get("nearest_idx", 0))
                self.frame += 1
                # detect_range: the LiDAR vehicle detector misses this car further away
                # (its returns stay in the static scan, as live runs 184/186/187 showed).
                # ghost: (idx, frame) -> shown; a perception false positive (no body, no LiDAR).
                ego_xy = np.array([self.x, self.y])
                # visible(t, frame): fused-camera flicker (run 264: the car ahead was listed
                # in a few frames only, from 5.5 m).
                objects = [a.sensed(t, ego_xy, self.yaw) for a in self.actors if a.active and not getattr(a, "lidar_only", False) and
                           math.hypot(*(a.xy - [self.x, self.y])) <= getattr(a, "detect_range", 1e9)
                           and (getattr(a, "ghost", None) is None or a.ghost(idx_now, self.frame))
                           and (getattr(a, "visible", None) is None or a.visible(t, self.frame))]
                objects = [o for o in objects if o is not None]
                rt.objects = [o for o in objects]
                rt.cars = [o for o in objects if o[4] == "npc"]
                rt.vehicle_ready = not any(t0 <= t < t1 for t0, t1 in self.vehicle_not_ready)
                r.vehicle_receiver.publish_rows(rt.cars, stamp=clock.now, ready=rt.vehicle_ready)
                r.object_receiver.received = clock.now
                r.lidar.pts = self.lidar_points()
                r.lidar.received = clock.now
                r.lidar._scan_stamp = clock.now
                rt.tunnel_scan = self.tunnel_scan(clock.now)
                self.yaw_bias = math.radians(float(self.heading_bias(idx_now))) if self.heading_bias else 0.
                rt.ego = dict(x=float(self.x), y=float(self.y), heading_deg=math.degrees(self.yaw + self.yaw_bias),
                              speed_kmh=self.v * 3.6, age_s=.01, stamp=clock.now, received=clock.now)
                began = time.perf_counter()
                r.infer()
                spent = time.perf_counter() - began
                pub = r.path_publisher
                target = float(pub.velocity_pub.last.data) if pub.velocity_pub.last is not None else 0.
                waypoints = json.loads(pub.waypoints_pub.last.data)["waypoints"] if pub.waypoints_pub.last else []
                if CURVE_FOLLOWER and target > 0.:
                    cap = self.follower_curve_cap(waypoints, int((r.latest_status.get("path") or {}).get("nearest_idx", 0)))
                    if cap is not None:
                        target = min(target, cap)
                record = getattr(r, "_web_record", {}) or {}
                lattice = record.get("lattice") or {}
                if lattice.get("outside_m") is not None:
                    outside = float(lattice["outside_m"])
                    stats["outside_max_m"] = max(stats["outside_max_m"], outside)
                    stats["outside_sum_m"] += outside
                    stats["outside_samples"] += 1
                mode = record.get("mode", "?")
                stats["modes"][mode] = stats["modes"].get(mode, 0) + 1
                # SIM_FIXED_DT=1: one decision per 1/15 s regardless of CPU load (repeatable A/B runs).
                step = DT if os.environ.get("SIM_FIXED_DT") == "1" else max(DT, spent)
                if self.decision_s:
                    step = max(step, float(self.decision_s))
                # The decision used the state at its start; the car moves meanwhile.
                substeps = max(1, int(round(step / DT)))
                if self.rtf:
                    substeps = max(1, int(round(step * self.rtf / DT)))
                    step = substeps * DT / self.rtf
                for sub in range(substeps):
                    if sub:
                        # Other road users keep moving during a slow decision too.
                        # Stepping them only once per decision slowed every mover by
                        # DT/step under CPU load (0.1 s decisions: a 31 km/h car
                        # really drove 21 km/h while reporting 31), so the ego ran
                        # into a "faster" lead (regression highway_random_3).
                        for actor in self.actors:
                            if getattr(actor, "follow_ego", False):
                                self.npc_follow_ego(actor)
                            actor.step(t + sub * DT, DT)
                    old = (self.x, self.y)
                    self.control(self.drifted(waypoints, r), target)
                    stats["distance"] += math.hypot(self.x - old[0], self.y - old[1])
                ego_box = box_corners(self.x, self.y, self.yaw, 2 * HALF_L, 2 * HALF_W)
                # true_body: MORAI's ego position is the rear-axle centre (judge.py, Ego Detail
                # wheels): the body reaches 3.845 m ahead and 0.79 m behind it, not +-2.3175.
                # People are checked against both boxes (auto runs 243/247: front-fender contacts).
                true_box = box_corners(self.x + TRUE_BODY_SHIFT * math.cos(self.yaw), self.y + TRUE_BODY_SHIFT * math.sin(self.yaw),
                                       self.yaw, 2 * HALF_L, 2 * HALF_W) if self.true_body else None
                for actor in self.actors:
                    if not actor.active or getattr(actor, "ghost", None) is not None:
                        continue
                    gap = box_gap(ego_box, actor.corners())
                    if true_box is not None and actor.kind == "pedestrian":
                        gap = min(gap, box_gap(true_box, actor.corners()))
                    stats["min_gap"] = min(stats["min_gap"], gap)
                    if gap <= 0. and stats["collision"] is None:
                        stats["collision"] = dict(t=round(t, 1), actor=actor.name,
                                                  ego=[round(self.x, 1), round(self.y, 1)], speed_kmh=round(self.v * 3.6, 1))
                clock.now += step
                stats["speeds"].append(self.v * 3.6)
                if self.v < .5:
                    current_stop += step
                    stats["stop_s"] += step
                    reason = record.get("stop_reason") or record.get("reason") or "?"
                    reason = str(reason)[:70]
                    stats["stop_reasons"][reason] = stats["stop_reasons"].get(reason, 0.) + step
                    stats["longest_stop_s"] = max(stats["longest_stop_s"], current_stop)
                else:
                    current_stop = 0.
                self.log.append((round(t, 2), round(self.x, 2), round(self.y, 2), round(self.v * 3.6, 1),
                                 round(target * 3.6, 1), mode, record.get("stop_reason") or record.get("reason"),
                                 round(self.yaw, 4), round(self.steer, 4),
                                 ((record.get("lattice") or {}).get("racing_gate")),
                                 {k: (record.get("lattice") or {}).get(k) for k in ("line", "rejected", "curvature", "line_cap_kmh", "kind", "anchor_m", "reason", "stitched")},
                                 record.get("ped_guard")))
                if stats["collision"]:
                    break
                if self.end_idx is not None:
                    idx = int((r.latest_status.get("path") or {}).get("nearest_idx", 0))
                    if idx >= self.end_idx:
                        stats["reached"] = True
                        break
            stats["t"] = clock.now - start
        finally:
            time.monotonic, time.time = real_monotonic, real_time
        sp = np.asarray(stats.pop("speeds"))
        stats["outside_mean_m"] = round(stats.pop("outside_sum_m") / max(1, stats.pop("outside_samples")), 3)
        stats["outside_max_m"] = round(stats["outside_max_m"], 3)
        stats["mean_kmh"] = round(float(sp.mean()), 1) if len(sp) else 0.
        stats["min_gap"] = round(stats["min_gap"], 2)
        stats["stop_s"] = round(stats["stop_s"], 1)
        stats["longest_stop_s"] = round(stats["longest_stop_s"], 1)
        stats["t"] = round(stats["t"], 1)
        stats["distance"] = round(stats["distance"], 1)
        stats["stop_reasons"] = {k: round(v, 1) for k, v in sorted(stats["stop_reasons"].items(), key=lambda kv: -kv[1])[:4]}
        stats["modes"] = dict(sorted(stats["modes"].items(), key=lambda kv: -kv[1])[:5])
        return stats


# --------------------------------------------------------------------------- scenarios
def load_world():
    waypoints = np.array([[p["x"], p["y"]] for p in json.loads((HERE / "global_path.json").read_text())["waypoints"]])
    lanes = HdMapLanes(HERE / "link_set.json")
    if hasattr(lanes, "set_route"):
        lanes.set_route(waypoints)
    return dict(waypoints=waypoints, smooth=smooth_control_route(waypoints), lanes=lanes)


def route_frame(world, idx, along=0., lateral=0.):
    """World xy and heading (deg) at route idx, shifted along/left of the route."""
    wp = world["smooth"]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(wp, axis=0), axis=1))]
    s = arc[idx] + along
    j = int(np.clip(np.searchsorted(arc, s), 1, len(wp) - 2))
    p = wp[j - 1] + (wp[j] - wp[j - 1]) * ((s - arc[j - 1]) / max(1e-6, arc[j] - arc[j - 1]))
    h = math.atan2(*(wp[j + 1] - wp[j - 1])[::-1])
    n = np.array([-math.sin(h), math.cos(h)])
    return p + n * lateral, math.degrees(h)


def route_polyline(world, idx, along0, along1, lateral=0., step=1.):
    return np.array([route_frame(world, idx, a, lateral)[0] for a in np.arange(along0, along1, step)])


def static(world, idx, name, along, lateral, length=1., width=1., height=1., kind="obstacle"):
    xy, h = route_frame(world, idx, along, lateral)
    return Actor(name, kind, xy, h, length, width, height)


def lane_polyline(world, idx, along, lateral, length, step=1.):
    """Map lane through route_frame(idx, along, lateral), followed downstream for length m.

    Unlike route_polyline this keeps the physical lane where the global route
    crosses lanes (highway merge at idx 2430-2600, work-zone shift 3060-3330)."""
    lanes = world["lanes"]
    start, heading = route_frame(world, idx, along, lateral)
    h = math.radians(heading)
    best = None
    for link_id, pts in lanes.geometry.items():
        d = np.linalg.norm(pts - start, axis=1)
        k = int(np.argmin(d))
        if d[k] > 2.5 or len(pts) < 2:
            continue
        seg = pts[min(k + 1, len(pts) - 1)] - pts[max(k - 1, 0)]
        dh = abs((math.atan2(seg[1], seg[0]) - h + math.pi) % (2 * math.pi) - math.pi)
        if dh < math.radians(20) and (best is None or d[k] < best[0]):
            best = (float(d[k]), link_id, k)
    if best is None:
        return route_polyline(world, idx, along, along + length, lateral, step)
    _, link_id, k = best
    out = [start, *lanes.geometry[link_id][k + 1:]]
    total = 0.
    while True:
        total = float(np.sum(np.linalg.norm(np.diff(np.asarray(out), axis=0), axis=1)))
        if total >= length:
            break
        link = lanes.link_by_id[link_id]
        pts = lanes.geometry[link_id]
        last = pts[-1] - pts[max(0, len(pts) - 4)]
        choices = []
        for nxt in lanes.successors.get(link.get("to_node_idx"), ()):
            q = lanes.geometry.get(nxt)
            if q is None or len(q) < 2:
                continue
            first = q[min(3, len(q) - 1)] - q[0]
            turn = abs((math.atan2(first[1], first[0]) - math.atan2(last[1], last[0]) + math.pi) % (2 * math.pi) - math.pi)
            # Prefer the lane that keeps its side position (parallel lanes), not a crossing link.
            choices.append((turn, nxt))
        if not choices:
            break
        link_id = min(choices)[1]
        out.extend(lanes.geometry[link_id][1:])
    out = np.asarray(out, float)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(out, axis=0), axis=1))]
    keep = np.r_[True, np.diff(arc) > 1e-6]
    out, arc = out[keep], arc[keep]
    s = np.arange(0., min(length, arc[-1]), step)
    return np.column_stack([np.interp(s, arc, out[:, i]) for i in (0, 1)])


def lane_car(world, name, idx, along, lateral, kmh, uid, length=4.6, width=1.9, run=900., start_time=0.):
    path = lane_polyline(world, idx, along, lateral, run)
    head = path[min(3, len(path) - 1)] - path[0]
    car = Actor(name, "npc", path[0], math.degrees(math.atan2(head[1], head[0])), length, width, 1.5,
                speed=kmh / 3.6, path=path, uid=uid, start_time=start_time)
    car.follow_ego = True
    return car


def exit_drum_actors(world):
    """Auto run 188 exit-taper drum lines (map scenery, LiDAR positions from the bag)."""
    wp = world["smooth"]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(wp, axis=0), axis=1))]
    start = 2900
    def at(idx, lat):
        return static(world, start, "Barrel", float(arc[idx] - arc[start]), lat, .6, .6, 1.)
    right = [at(i, -1.5 + .0015 * (i - 3060)) for i in range(3060, 3112, 5)]
    left = [at(i, 2.95 - .01 * (i - 3045)) for i in range(3045, 3082, 5)]
    far = [at(i, -3.9 - .02 * (i - 3106)) for i in range(3106, 3126, 5)]
    return right + left + far


def highway_random(world, seed):
    """Whole highway (idx 2252-3500) with competition-like NPC traffic: mostly slow
    (20-45 km/h) cars in every lane, sometimes a stopped car, plus the exit drums."""
    rng = np.random.default_rng(seed)
    start = 2252
    actors, uid = [], 960
    for lane_k in (0, 1, 2, 3):
        along = float(rng.uniform(-40., 30.))
        while along < 560.:
            stopped = rng.random() < .08 and 120. < along < 320.
            kmh = 0. if stopped else float(rng.uniform(20., 45.))
            lat = 3.7 * lane_k
            if not (lane_k == 0 and abs(along) < 25.):
                if stopped:
                    xy, h = route_frame(world, start, along, lat)
                    actors.append(Actor("StoppedCar", "npc", xy, h, 4.6, 1.9, 1.5, uid=uid))
                else:
                    actors.append(lane_car(world, f"npc{uid}", start, along, lat, kmh, uid))
                uid += 1
            along += float(rng.uniform(45., 140.))
    drums = exit_drum_actors(world)
    for drum in drums:
        # Live runs 186/188/189 in the taper: 0-3 fused objects for ~20 drums, so
        # the drums reach the planner as LiDAR returns only.
        drum.detect_range = 0.
    return Sim(world, start, speed_kmh=44., end_idx=3500, duration_s=110., stop_line=False,
               actors=actors + drums)


def scenarios(world):
    hw0 = 2300      # highway section 2251..3500
    nr0 = 800       # inside the obstacle mission (idx 480-1100)
    out = {}

    # Run 407: two close, alternating obstacles at 39 km/h. ext=.55 m
    # is the half extent, so the sensed boxes are 1.1 m wide.
    def slalom407(gap=7., swapped=False, triple=False, recorded=False):
        lateral = (.4, -2.4) if swapped else (-2.4, .4)
        if recorded:
            lateral = (-1.57, 1.42)  # logged route lat; f/l use the ego frame
        placements = [(15., lateral[0]), (15. + gap, lateral[1])]
        if triple:
            placements.append((15. + 2. * gap, lateral[0]))
        return Sim(world, 716, speed_kmh=39., end_idx=816, duration_s=60.,
                   stop_line=False, highway=False, zone_kmh=39.,
                   actors=[static(world, 716, "Slalom407", a, l, 1.1, 1.1, 1.)
                           for a, l in placements])
    out["slalom_407_7m"] = slalom407
    out["slalom_407_5m"] = lambda: slalom407(gap=5.)
    out["slalom_407_10m"] = lambda: slalom407(gap=10.)
    out["slalom_407_swapped"] = lambda: slalom407(swapped=True)
    out["slalom_407_triple"] = lambda: slalom407(triple=True)
    out["slalom_407_recorded"] = lambda: slalom407(recorded=True)
    out["slalom_407_morai_four"] = lambda: Sim(
        world, 665, speed_kmh=39., end_idx=840, duration_s=60., stop_line=False,
        highway=False, zone_kmh=39.,
        actors=[Actor("Slalom407", "obstacle", (x, y), 90., 1.1, 1.1, 1.) for x, y in
                [(-58.21, -99.3), (-61.31, -92.4), (-58.53, -81.8), (-61.40, -71.2)]])

    out["normal_box_on_route"] = lambda: Sim(world, nr0, speed_kmh=30., end_idx=nr0 + 200, duration_s=60., stop_line=False, highway=False,
        actors=[static(world, nr0, "CargoBox", 45., 0., 1.2, 1.2, 1.0)])
    out["normal_barrels_chicane"] = lambda: Sim(world, nr0, speed_kmh=30., end_idx=nr0 + 260, duration_s=80., stop_line=False, highway=False,
        actors=[static(world, nr0, "Barrel", 40., .4, .6, .6, .9), static(world, nr0, "Barrel", 70., -.5, .6, .6, .9)])
    out["normal_parked_car_edge"] = lambda: Sim(world, nr0, speed_kmh=30., end_idx=nr0 + 200, duration_s=60., stop_line=False, highway=False,
        actors=[static(world, nr0, "ParkedCar", 45., 1.2, 4.6, 1.9, 1.5, kind="obstacle")])
    out["normal_pedestrian_crossing"] = lambda: Sim(world, nr0, speed_kmh=30., end_idx=nr0 + 200, duration_s=60., stop_line=False, highway=False,
        actors=[Actor("person", "pedestrian", route_frame(world, nr0, 35., -6.)[0], 90., .6, .6, 1.7, speed=1.2,
                      path=[route_frame(world, nr0, 35., -6.)[0], route_frame(world, nr0, 35., 6.)[0]], uid=900)])
    # A person standing still at the lane edge, half in our lane (live
    # 2026-10-03: stood 20+ s). Expect a wait of ~5 s, then a pass around.
    out["normal_pedestrian_standing"] = lambda: Sim(world, nr0, speed_kmh=30., end_idx=nr0 + 200, duration_s=60., stop_line=False, highway=False,
        actors=[Actor("person", "pedestrian", route_frame(world, nr0, 35., -1.3)[0], 90., .6, .6, 1.7, speed=0.,
                      path=[route_frame(world, nr0, 35., -1.3)[0]], uid=902)])
    for seed in range(int(os.environ.get("SIM_RANDOM_N", "20"))):
        out[f"random_box_{seed}"] = (lambda seed=seed: random_box(world, seed))
    out["normal_slow_lead_car"] = lambda: Sim(world, nr0, speed_kmh=30., end_idx=nr0 + 240, duration_s=60., stop_line=False, highway=False,
        actors=[Actor("lead", "npc", route_frame(world, nr0, 25.)[0], 0., 4.6, 1.9, 1.5, speed=4.,
                      path=route_polyline(world, nr0, 25., 400.), uid=901)])

    # ---- Live run 264 (t 56-62 s, idx 330-410): a car ahead in our lane at ~35 km/h slowed
    # into the queue 10 m before the first signal junction (stop line idx 420) while we came
    # at 50-53 km/h (zone 55). Outside the surveyed junctions the LiDAR vehicle detector is
    # off and the fused camera car came and went (first listed 5.5 m ahead), so the car
    # reached the planner as LiDAR returns only: contact at 29.5 km/h. Runs 166/244/254 then
    # stood 56-60+ s behind that queue.
    def run264(*, stand_s=30., fusion=None, lead_kmh=35., start_kmh=50., stop_idx=405, start_idx=290,
               lead_along=25., resume_kmh=25., bias=None):
        def build():
            path = route_polyline(world, start_idx, lead_along, 260.)
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
            wp = world["smooth"]
            stop_s = float(arc[int(np.argmin(np.linalg.norm(path - wp[stop_idx], axis=1)))])
            car = QueueCar("queue_lead", path, lead_kmh, stop_s, uid=930,
                           resume_t=None if stand_s is None else stop_s / (lead_kmh / 3.6) + 4. + stand_s,
                           resume_kmh=resume_kmh)
            car.lidar_range = 30.
            if fusion is None:
                car.detect_range = 0.            # LiDAR returns only
            else:
                car.detect_range, car.visible = fusion
            sim = Sim(world, start_idx, speed_kmh=start_kmh, end_idx=480, duration_s=110., stop_line=False,
                      highway=False, actors=[car], vehicle_not_ready=((0., 1e9),), heading_bias=bias)
            sim.rt.r.speed_zones = SimpleNamespace(kmh_at=lambda index: (55., {"kmh": 55.}))
            return sim
        return build
    out["normal_264_queue_lidar_only"] = run264()
    # The fused car listed only within 12 m and in 3 of every 10 frames (detector late).
    out["normal_264_queue_fusion_late"] = run264(fusion=(12., lambda t, frame: frame % 10 < 3))
    # The same with the 3 deg localization heading bias measured in run 264 (map corridor
    # swings 1.6 m off at 30 m).
    out["normal_264_queue_heading_bias"] = run264(bias=lambda idx: -3.)
    # A slow car (15 km/h) that never stops: follow it at a safe gap through the junction.
    out["normal_264_slow_lead_follow"] = run264(stand_s=0., lead_kmh=15., stop_idx=470, resume_kmh=15.)
    # Worst case: the car already stands in the queue, LiDAR returns from 30 m only, we
    # arrive at 55 km/h (stopping needs ~59 m at the planned 2.5 m/s^2).
    out["normal_264_standing_car_30m"] = run264(lead_kmh=0.01, start_kmh=55., lead_along=57., stand_s=25.)
    # A car standing for good in our lane where there is a second same-direction lane
    # (idx ~1340, 250 m from the next junction): not a queue -> wait, then plan around it.
    def broken_down():
        car = static(world, 1280, "BrokenDownCar", 70., 0., 4.6, 1.9, 1.5, kind="npc")
        car.detect_range, car.lidar_range = 0., 30.
        sim = Sim(world, 1280, speed_kmh=40., end_idx=1500, duration_s=110., stop_line=False, highway=False,
                  actors=[car], vehicle_not_ready=((0., 1e9),), model_points=12)
        sim.rt.r.speed_zones = SimpleNamespace(kmh_at=lambda index: (55., {"kmh": 55.}))
        return sim
    out["normal_broken_down_car"] = broken_down

    # Auto run 102: a right bypass met a person walking toward us on the right shoulder.
    out["normal_box_shoulder_walker"] = lambda: Sim(world, nr0, speed_kmh=30., end_idx=nr0 + 200, duration_s=60.,
        stop_line=False, highway=False,
        actors=[static(world, nr0, "CargoBox", 45., .6, 1.2, 1.2, 1.0),
                Actor("walker", "pedestrian", route_frame(world, nr0, 70., -3.0)[0], 180., .6, .6, 1.7, speed=1.3,
                      path=[route_frame(world, nr0, 70., -3.0)[0], route_frame(world, nr0, 20., -3.0)[0]], uid=903)])
    # Auto run 107: a MORAI pedestrian walked into our stopped car's front-right corner
    # and stood there (they never avoid us); we waited for it for good.
    out["normal_pedestrian_pins_bumper"] = lambda: Sim(world, nr0, speed_kmh=0., end_idx=nr0 + 140, duration_s=60.,
        stop_line=False, highway=False,
        actors=[Actor("pinner", "pedestrian", route_frame(world, nr0, 6., 4.0)[0], 180., .6, .6, 1.7, speed=1.0,
                      path=[route_frame(world, nr0, 6., 4.0)[0], route_frame(world, nr0, 2.9, 1.2)[0]], uid=904)])
    # Auto run 138: a person beside our body drifting toward us while we stop-go
    # beside them; they walked into our side at 1-2 km/h. Expect: drive on past.
    out["normal_pedestrian_beside_drift"] = lambda: Sim(world, nr0, speed_kmh=12., end_idx=nr0 + 120, duration_s=45.,
        stop_line=False, highway=False,
        actors=[Actor("drifter", "pedestrian", route_frame(world, nr0, 9., 2.6)[0], 180., .6, .6, 1.7, speed=.6,
                      path=[route_frame(world, nr0, 9., 2.6)[0], route_frame(world, nr0, 7., .2)[0]], uid=905)])
    # Auto run 145: a person standing still in our lane (0.5 m left of centre), a box
    # on the right shoulder; 31 s "no collision-free path", reverse vetoed by a return
    # behind. Expect: wait 5 s, then pass around on the left.
    out["normal_pedestrian_standing_in_lane"] = lambda: Sim(world, nr0, speed_kmh=25., end_idx=nr0 + 160, duration_s=60.,
        stop_line=False, highway=False,
        actors=[static(world, nr0, "CargoBox", 36., -4.0, 1.2, 1.2, 1.0),
                Actor("stander", "pedestrian", route_frame(world, nr0, 42., .5)[0], 180., .6, .6, 1.7, uid=906)])
    # Auto run 156: a mixer truck (9 m x 2.5 m) filling our lane in the box section;
    # stopped 4 m short, three reverses, 1 Hz decisions (800 ms tight re-plans), stuck.
    out["normal_mixer_truck_in_lane"] = lambda: Sim(world, nr0, speed_kmh=30., end_idx=nr0 + 160, duration_s=60.,
        stop_line=False, highway=False,
        actors=[static(world, nr0, "MixerTruck", 45., -.6, 9.2, 2.5, 3.5)])
    # A person standing 2.3 m beside our path: room to pass. Expect no stop, pass slowly
    # (user 2026-10-05: stop the stop-go when there is room).
    out["normal_pedestrian_beside_standing"] = lambda: Sim(world, nr0, speed_kmh=25., end_idx=nr0 + 140, duration_s=40.,
        stop_line=False, highway=False,
        actors=[Actor("bystander", "pedestrian", route_frame(world, nr0, 40., 2.3)[0], 180., .6, .6, 1.7, uid=907)])
    # Auto run 186: a person walking along the shoulder 2.9 m beside our path at our
    # crawl speed stayed ~6 m ahead; the caution cap held us at 2 km/h for 35 s.
    out["normal_pedestrian_walking_beside"] = lambda: Sim(world, nr0, speed_kmh=4., end_idx=nr0 + 120, duration_s=45.,
        stop_line=False, highway=False,
        actors=[Actor("shoulder_walker", "pedestrian", route_frame(world, nr0, 8., 2.9)[0], 0., .6, .6, 1.7,
                      speed=.55, path=route_polyline(world, nr0, 8., 40., 2.9), uid=908)])
    # Auto run 188: drum lines at the highway exit taper (map scenery, LiDAR
    # positions from the bag). Cruise at 85 km/h kept until 1.5 s before the drums
    # 0.3 m beside the body; the late full brake still knocked them over.
    def exit_drums():
        wp = world["smooth"]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(wp, axis=0), axis=1))]
        start = 2900
        def at(idx, lat):
            return static(world, start, "Barrel", float(arc[idx] - arc[start]), lat, .6, .6, 1.)
        right = [at(i, -1.5 + .0015 * (i - 3060)) for i in range(3060, 3112, 5)]
        left = [at(i, 2.95 - .01 * (i - 3045)) for i in range(3045, 3082, 5)]
        far = [at(i, -3.9 - .02 * (i - 3106)) for i in range(3106, 3126, 5)]
        return Sim(world, start, speed_kmh=85., end_idx=3200, duration_s=40., stop_line=False,
                   actors=right + left + far)
    out["highway_exit_drum_taper"] = exit_drums
    # Auto run 206: a shoulder walker 2.1 m right of the path, 0.3 m/s, ~5 m ahead: the
    # caution cap (2.2 m band) held us at 2 km/h behind them.
    out["normal_pedestrian_walking_beside_close"] = lambda: Sim(world, nr0, speed_kmh=4., end_idx=nr0 + 120, duration_s=45.,
        stop_line=False, highway=False,
        actors=[Actor("shoulder_walker_close", "pedestrian", route_frame(world, nr0, 6., -2.1)[0], 0., .6, .6, 1.7,
                      speed=.3, path=route_polyline(world, nr0, 6., 30., -2.1), uid=909)])
    # Auto runs 243 / 247 (MORAI pedestrian contacts, ids 10019 / 10013): people crossing
    # our lane at ~0.45 m/s, positions from the fused tracks and the Collision Data in
    # the bags. Fusion saw them only 11-16 m ahead, with velocity 0 on half the frames;
    # decisions ran at ~4 Hz. Both times we braked hard and stopped across their line.
    # MORAI ran at 0.55 (243) / 0.63 (247) of real time (ego_status distance vs speed), so
    # the 0.45 m/s (wall clock) walkers are 0.82 / 0.71 m/s of simulator time (rtf below).
    def ped(name, uid, xy0, xy1, kmh, rng, start_s=0., within=None):
        a = Actor(name, "pedestrian", xy0, 0., .6, .6, 1.7, speed=kmh / 3.6, path=None if xy1 is None else [xy0, xy1],
                  uid=uid, start_time=start_s)
        a.ped_noise = dict(range_m=rng, drop_p=.25, v_zero_p=.5, pos_sigma=.12)
        if within is not None:
            a.start_within_m = within
        return a
    out["normal_pedestrian_turns_in_243"] = lambda: Sim(world, 583, speed_kmh=39.3, end_idx=700, duration_s=60.,
        stop_line=False, highway=False, true_body=True, decision_s=.25, rtf=.55, zone_kmh=48.87,
        actors=[ped("crosser_243", 916, (-55.08, -137.95), (-68., -137.95), 2.95, 15.5),
                ped("left_crosser_243", 918, (-64.2, -131.9), (-52., -131.9), 2.6, 21.),
                ped("left_walker_243", 914, (-62.85, -149.5), (-62.85, -110.), 4.6, 29.)])
    out["normal_pedestrian_crossing_late_247"] = lambda: Sim(world, 858, speed_kmh=24.4, end_idx=950, duration_s=60.,
        stop_line=False, highway=False, true_body=True, decision_s=.22, rtf=.63, zone_kmh=48.87, lateral_offset=-1.,
        actors=[ped("crosser_247", 920, (-56.63, -26.6), (-68., -26.6), 2.57, 12.),
                ped("stander_247", 923, (-57.33, -23.3), None, 0., 12.)])
    # The same person standing 3 m right of the path until we are 11 m away (rear
    # axle), then crossing at 0.5 m/s: the "standing, then walks" reading of run 247.
    out["normal_pedestrian_starts_walking"] = lambda: Sim(world, 858, speed_kmh=24.4, end_idx=950, duration_s=60.,
        stop_line=False, highway=False, true_body=True, decision_s=.22, rtf=.63, zone_kmh=48.87,
        actors=[ped("starter", 921, (-57.1, -26.6), (-68., -26.6), 2.9, 14., within=11.)])
    # Auto run 268 (14 contacts, 210 s): stopped at the end of the box section behind a
    # person standing in the lane between boxes, a walker fusion never listed came up our
    # right side from behind and stayed 0.05-0.1 m from the rear half of the body; reverse
    # escapes and creeps moved us along them again and again. Expect: never reverse or
    # creep into the walker; leave forward once the person ahead has gone.
    def run_268(walker_from_behind=False):
        base = 940
        blocker_xy = route_frame(world, base, 9., .3)[0]
        blocker = Actor("lane_stander_268", "pedestrian", blocker_xy, 0., .6, .6, 1.7, speed=.8,
                        path=[blocker_xy, route_frame(world, base, 9., 7.)[0]], uid=931, start_time=26.)
        if walker_from_behind:
            side = Actor("side_walker_268", "pedestrian", route_frame(world, base, -7., -1.4)[0], 0., .6, .6, 1.7,
                         speed=.8, path=route_polyline(world, base, -7., -.1, -1.4, step=.5), uid=932)
            side.persist = True
        else:   # the state of run 268 from t=193 s: standing 0.05 m from our right side by the rear axle
            side = Actor("side_stander_268", "pedestrian", route_frame(world, base, -.15, -1.3)[0], 0., .6, .6, 1.7, uid=932)
        side.lidar_only = True                                 # never in the fused object list
        return Sim(world, base, speed_kmh=0., end_idx=base + 90, duration_s=90., stop_line=False, highway=False,
                   true_body=True, decision_s=.25, rtf=.55, zone_kmh=48.87,
                   actors=[blocker, side, static(world, base, "CargoBox", 9., 2.7, 1.2, 1.2, 1.0),
                           static(world, base, "CargoBox", 9., -2.7, 1.2, 1.2, 1.0)])
    out["normal_pedestrian_contact_268"] = run_268
    out["normal_pedestrian_contact_268_walker"] = lambda: run_268(walker_from_behind=True)
    out["normal_single_lane_box_edge"] = lambda: Sim(world, 700, speed_kmh=30., end_idx=900, duration_s=60., stop_line=False, highway=False,
        actors=[static(world, 700, "CargoBox", 45., 1.3, 1.0, 1.0, 1.0)])
    out["normal_staggered"] = lambda: Sim(world, nr0, speed_kmh=30., end_idx=nr0 + 220, duration_s=60., stop_line=False, highway=False,
        actors=[static(world, nr0, "Barrel", 40., -1.2, .6, .6, .9), static(world, nr0, "Barrel", 52., 1.4, .6, .6, .9),
                static(world, nr0, "CargoBox", 66., -.8, 1.0, 1.0, 1.0)])
    out["normal_phantom_model"] = lambda: Sim(world, nr0, speed_kmh=30., end_idx=nr0 + 200, duration_s=40., stop_line=False, highway=False,
        model_offset_m=2.5)

    def highway(actors, duration=70., start_kmh=60.):
        return lambda: Sim(world, hw0, speed_kmh=start_kmh, end_idx=hw0 + 700, duration_s=duration,
                           stop_line=False, actors=actors())
    lane = 3.5
    # Auto run 83: our lane ends at the merge (~idx 2753) while a car closes from
    # behind in the left lane at 35 km/h; we waited at the taper end and were clipped.
    def merge(behind_m, npc_kmh):
        return lambda: Sim(world, 2600, speed_kmh=30., end_idx=2950, duration_s=60., stop_line=False,
            actors=[Actor("merge_npc", "npc", route_frame(world, 2600, -behind_m, 3.5)[0], 0., 4.6, 1.9, 1.5,
                          speed=npc_kmh / 3.6, path=route_polyline(world, 2600, -behind_m, 400., 3.5), uid=950)])
    out["highway_merge_car_behind"] = merge(20., 35.)
    out["highway_merge_car_beside"] = merge(3., 30.)
    # Auto runs 112/119: a merging car came to a stop at our left rear, its box
    # overlapping ours; we stood beside it for good. Both start at 0 km/h.
    out["highway_stopped_beside_rear"] = lambda: Sim(world, 2600, speed_kmh=0., end_idx=2800, duration_s=40., stop_line=False,
        actors=[Actor("merged", "npc", route_frame(world, 2600, -4., 1.9)[0], 0., 4.6, 1.9, 1.5, uid=951)])
    out["highway_empty"] = highway(lambda: [])
    out["highway_roadside_cones"] = highway(lambda: [static(world, hw0, "cone", a, -(HALF_W + .25 + .2), .4, .4, .7)
                                                     for a in range(30, 200, 8)])
    out["highway_stopped_car_in_lane"] = highway(lambda: [static(world, hw0, "StoppedCar", 120., 0., 4.6, 1.9, 1.5)])
    out["highway_slow_car_ahead"] = highway(lambda: [Actor("slow", "npc", route_frame(world, hw0, 60.)[0], 0., 4.6, 1.9, 1.5,
                                                           speed=40 / 3.6, path=route_polyline(world, hw0, 60., 600.), uid=910)])
    out["highway_adjacent_traffic"] = highway(lambda: [
        Actor("adjL", "npc", route_frame(world, hw0, -15., lane)[0], 0., 4.6, 1.9, 1.5, speed=75 / 3.6,
              path=route_polyline(world, hw0, -15., 600., lane), uid=911),
        Actor("adjR", "npc", route_frame(world, hw0, 20., -lane)[0], 0., 4.6, 1.9, 1.5, speed=70 / 3.6,
              path=route_polyline(world, hw0, 20., 600., -lane), uid=912),
        static(world, hw0, "StoppedCar", 150., 0., 4.6, 1.9, 1.5)])

    # ---- whole-highway flows from the auto runs 183-190 (entry at 44 km/h, idx 2252).
    # Runs 186/187: a 25 km/h car in the left lane that our route merges into
    # (idx 2430-2600); "imminent predicted collision" stopped us to 0 km/h at idx 2503.
    out["highway_merge_slow_car"] = lambda: Sim(world, 2252, speed_kmh=44., end_idx=3000, duration_s=80.,
        stop_line=False, actors=[lane_car(world, "slow_merge", 2252, 38., 3.7, 25., 970),
                                 lane_car(world, "left2", 2252, -15., 7.4, 30., 971)])
    # The same with the slow car further ahead: we catch it inside the merge taper.
    out["highway_merge_slow_car_far"] = lambda: Sim(world, 2252, speed_kmh=44., end_idx=3000, duration_s=80.,
        stop_line=False, actors=[lane_car(world, "slow_merge", 2252, 80., 3.7, 30., 975),
                                 lane_car(world, "left2", 2252, 60., 7.4, 30., 976)])
    # Runs 186/187: a car ~45 m ahead in the out lane that the vehicle detector
    # did not list; its LiDAR returns read as a static obstacle (50 km/h for 15 s).
    def undetected_lead():
        car = lane_car(world, "far_lead", 2520, 60., 3.6, 55., 985)
        car.detect_range = 40.
        return Sim(world, 2520, speed_kmh=70., end_idx=3000, duration_s=70., stop_line=False, actors=[car])
    out["highway_undetected_lead"] = undetected_lead
    # A faster car (110 km/h) closing from behind in the out lane: change ahead of it
    # only with a TTC/no-hard-brake margin, else let it pass and change behind it,
    # without braking for it.
    out["highway_fast_rear_car"] = lambda: Sim(world, 2560, speed_kmh=85., end_idx=3150, duration_s=50.,
        stop_line=False, actors=[lane_car(world, "fast_rear", 2560, -35., 3.6, 110., 986)])
    # A 95 km/h car 15 m behind in our own lane while we wait for / make the change.
    out["highway_rear_tailgater"] = lambda: Sim(world, 2560, speed_kmh=85., end_idx=3150, duration_s=50.,
        stop_line=False, actors=[lane_car(world, "tailgater", 2560, -15., 0., 95., 987),
                                 lane_car(world, "side_slow", 2560, 10., 3.6, 70., 988)])
    # Auto run 243 (t=439-446 s, idx 2990-3070): a 29 km/h car in the left lane, which
    # the exit-taper drum line closes, merged into our lane (v_lat -0.2..-0.35) while we
    # drove past at 78 km/h; "imminent predicted collision" 75 -> 13 km/h, 0.65 m.
    def side_merge_taper():
        drums = exit_drum_actors(world)
        for drum in drums:
            drum.detect_range = 0.
        car = lane_car(world, "taper_merger", 2930, 45., 3.6, 29., 989)
        return Sim(world, 2930, speed_kmh=80., end_idx=3250, duration_s=50., stop_line=False,
                   actors=[car] + drums)
    out["highway_side_merge_taper"] = side_merge_taper
    # Plain overtake of a 30 km/h car in the next lane (no merge, no drift): the pass
    # speed is limited to 40 km/h above it, otherwise cruise.
    out["highway_side_slow_pass"] = lambda: Sim(world, 2620, speed_kmh=88., end_idx=2900, duration_s=30.,
        stop_line=False, actors=[lane_car(world, "side_slow", 2620, 70., 3.6, 30., 990)])
    # A 35 km/h car in the next lane cutting into ours ~0.7 m/s ahead of us.
    def side_cut_in():
        lat = lambda a: 3.6 if a < 95. else max(0., 3.6 - (a - 95.) * 3.6 / 50.)
        path = np.array([route_frame(world, 2800, a, lat(a))[0] for a in np.arange(70., 400., 1.)])
        car = Actor("cut_in", "npc", path[0], 0., 4.6, 1.9, 1.5, speed=35. / 3.6, path=path, uid=991)
        car.follow_ego = True
        return Sim(world, 2800, speed_kmh=88., end_idx=3040, duration_s=30., stop_line=False, actors=[car])
    out["highway_side_cut_in"] = side_cut_in
    # Auto run 266 (also 241): the car lagged the route's left shift in the on-ramp
    # merge (idx 2440-2600), ended 1.9 m right of the route, never re-centred (each
    # frame re-planned a 112 m join from where it was) and drove on the lane line to
    # the exit-taper drums, where it stood for 166 s at idx 3045.
    def offset_start():
        drums = exit_drum_actors(world)
        for drum in drums:
            drum.detect_range = 0.
        return Sim(world, 2560, speed_kmh=45., end_idx=3250, duration_s=60., stop_line=False,
                   lateral_offset=-1.9, actors=drums)
    out["highway_offset_lane_line"] = offset_start
    # Trapped in the lane the taper closes (left of the drum line) next to a queue
    # creeping at 2.5 m/s in the open lane: low-speed zipper merge into a ~9 m gap
    # (14 m spacing); with the run-266 spacing (10 m, ~5 m gaps) wait for the tail.
    def taper_queue(spacing):
        def build():
            drums = exit_drum_actors(world)
            for drum in drums:
                drum.detect_range = 0.
            queue = [lane_car(world, f"queue{k}", 2990, -40. + k * spacing, 0., 9., 992 + k)
                     for k in range(int(80. // spacing) + 1)]
            return Sim(world, 2990, speed_kmh=12., end_idx=3250, duration_s=120., stop_line=False,
                       lateral_offset=3.6, actors=queue + drums)
        return build
    out["highway_taper_queue_zipper"] = taper_queue(14.)
    out["highway_taper_queue_tight"] = taper_queue(10.)
    # Left lane busy: a row of 30 km/h cars every 30-40 m beside and ahead of us, so the
    # lane change has to be timed into a gap while we keep moving.
    out["highway_adjacent_platoon"] = lambda: Sim(world, 2520, speed_kmh=70., end_idx=3000, duration_s=70.,
        stop_line=False, actors=[lane_car(world, f"row{k}", 2520, a, 3.6, 30., 972 + k)
                                 for k, a in enumerate((-20., 12., 45., 82., 115., 150.))])
    # Slow car ahead in our lane and a slow car in the next lane slightly further ahead.
    out["highway_slow_lead_and_side"] = lambda: Sim(world, 2600, speed_kmh=80., end_idx=3000, duration_s=70.,
        stop_line=False, actors=[lane_car(world, "lead", 2600, 50., 0., 30., 980),
                                 lane_car(world, "side", 2600, 75., 3.6, 35., 981),
                                 lane_car(world, "side2", 2600, 20., 7.1, 40., 982)])
    # Runs 186/187: in the out lane a car ahead held us at ~50 km/h for 15 s.
    out["highway_out_lane_lead"] = lambda: Sim(world, 2520, speed_kmh=70., end_idx=3000, duration_s=70.,
        stop_line=False, actors=[lane_car(world, "outlead", 2520, 55., 3.6, 50., 983),
                                 lane_car(world, "routelead", 2520, 140., 0., 30., 984)])
    # Whole highway, no traffic: two lane changes, then the work-zone shift / last curve
    # (idx 3200 kink) and the exit (<= 60 km/h from idx 3500).
    out["highway_full_empty"] = lambda: Sim(world, 2252, speed_kmh=44., end_idx=3500, duration_s=90., stop_line=False)
    out["highway_full_exit_drums"] = lambda: Sim(world, 2252, speed_kmh=44., end_idx=3500, duration_s=90.,
        stop_line=False, actors=exit_drum_actors(world))
    for seed in range(int(os.environ.get("SIM_HIGHWAY_RANDOM_N", "4"))):
        out[f"highway_random_{seed}"] = (lambda seed=seed: highway_random(world, seed))

    # Junction A1256W000709: stop line at route station ~885 m.
    lanes = world["lanes"]
    station = np.asarray(getattr(lanes, "route_station", np.zeros(1)))
    stop_idx = int(np.argmin(np.abs(station - 885.1))) if len(station) > 1 else 1781
    def junction(headway, kmh=30., count=12):
        def build():
            # Cross traffic on the map link crossing the route closest beyond the stop line.
            cross = crossing_link(world, stop_idx)
            actors = []
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(cross, axis=0), axis=1))]
            for k in range(count):
                actors.append(Actor(f"cross{k}", "npc", cross[0], 0., 4.6, 1.9, 1.5, speed=kmh / 3.6,
                                    path=cross, uid=920 + k, start_time=k * headway))
            return Sim(world, stop_idx - 60, speed_kmh=20., end_idx=stop_idx + 60, duration_s=count * headway + 40.,
                       highway=False, actors=actors)
        return build
    out["intersection_empty"] = junction(1e6, count=1)
    out["intersection_6s"] = junction(6.)
    out["intersection_4s"] = junction(4.)
    out["intersection_3s"] = junction(3.)
    # Auto run 200: one slow car circling the roundabout crossed 3-5 m ahead just after
    # we entered; "clear it fast" turned the stop into 45 km/h. Arrival-time sweep.
    def junction_one(arrive_s, kmh=15.):
        def build():
            cross = crossing_link(world, stop_idx)
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(cross, axis=0), axis=1))]
            # Conflict: where the crossing link passes closest to our route just past the line.
            ours = world["smooth"][stop_idx:stop_idx + 30]
            c = int(np.argmin(np.min(np.linalg.norm(cross[:, None, :] - ours[None, :, :], axis=2), axis=1)))
            start = arc[c] - kmh / 3.6 * arrive_s
            pts = np.array([np.interp(a, arc, cross[:, 0]) for a in np.arange(start, arc[-1], 1.)])
            pts = np.column_stack((pts, [np.interp(a, arc, cross[:, 1]) for a in np.arange(start, arc[-1], 1.)]))
            return Sim(world, stop_idx - 60, speed_kmh=20., end_idx=stop_idx + 60, duration_s=60.,
                       highway=False, actors=[Actor("circling", "npc", pts[0], 0., 4.6, 1.9, 1.5,
                                                   speed=kmh / 3.6, path=pts, uid=940)])
        return build
    for k in (8, 9, 10, 11, 12, 13, 14, 15, 16, 17):
        out[f"intersection_cross_ahead_{k}"] = junction_one(float(k))

    # ---- Roundabout entry (competition "끼어들기" mission), live runs 200/210/211/222.
    # The circulating lane (link A2256W000134) comes from our left and merges with our
    # entry lane 15-19 m past the stop line (centre separation 4.8 m at +3.5 m, 2.5 m at
    # +6.5 m). The circulating car brakes for us only once we are in its lane (MORAI NPC).
    # Tracker faults as measured at this junction: a car vanishes for 0.9-2.6 s when it
    # is ~10 m from us and returns under a new id; every new track reports no valid
    # velocity (v_ok False, speed 0) for 0.36-0.55 s.
    def circulating_car(name, uid, arrive_s, kmh, *, start_xy=None, noise=None, resume=None, profile=None):
        cross = crossing_link(world, stop_idx)
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(cross, axis=0), axis=1))]
        ours = world["smooth"][stop_idx:stop_idx + 30]
        c = int(np.argmin(np.min(np.linalg.norm(cross[:, None, :] - ours[None, :, :], axis=2), axis=1)))
        start = (arc[c] - kmh / 3.6 * arrive_s if start_xy is None else
                 float(arc[int(np.argmin(np.linalg.norm(cross - np.asarray(start_xy), axis=1)))]))
        s = np.arange(start, arc[-1], 1.)
        pts = np.column_stack([np.interp(s, arc, cross[:, i]) for i in (0, 1)])
        car = Actor(name, "npc", pts[0], 0., 4.6, 1.9, 1.5, speed=kmh / 3.6, path=pts, uid=uid)
        car.follow_ego = True
        car.detect_range = 30.
        if resume is not None:          # (t, kmh): crawls (yielding to us), then drives on
            t_resume, kmh_resume = resume
            car.cruise_profile = lambda t, a=kmh / 3.6, b=kmh_resume / 3.6, t0=t_resume: a if t < t0 else b
        if profile is not None:         # [(t, kmh), ...]: cruise steps (a merged lead braking)
            steps = sorted(profile)
            car.cruise_profile = lambda t, a=kmh / 3.6: next((v / 3.6 for t0, v in reversed(steps) if t >= t0), a)
        if noise is not None:
            car.noise = dict(dict(drop_within_m=12., drop_s=1.2, drops=1, vok_delay_s=.5), **noise)
            car.noise["drops_left"] = car.noise.pop("drops")
        return car

    def roundabout(arrivals, kmh=12., *, noise=None, not_ready=(), start_pose=None, start_kmh=20.,
                   duration=60., resume=None, start_xy=None, profile=None):
        def build():
            cars = [circulating_car(f"circ{k}", 990 + k, a, kmh, noise=noise, resume=resume,
                                    start_xy=start_xy, profile=profile if k == 0 else None)
                    for k, a in enumerate(arrivals)]
            if start_pose is not None:
                return Sim(world, stop_idx + 4, speed_kmh=0., end_idx=stop_idx + 60, duration_s=duration,
                           highway=False, actors=cars, start_pose=start_pose, vehicle_not_ready=not_ready,
                           launch_lag=True)
            return Sim(world, stop_idx - 60, speed_kmh=start_kmh, end_idx=stop_idx + 60, duration_s=duration,
                       highway=False, actors=cars, vehicle_not_ready=not_ready, launch_lag=True)
        return build
    faults = dict(drop_within_m=12., drop_s=1.2, drops=1, vok_delay_s=.5)
    # One circulating car, tracker dropout + new-track v_ok False; arrival sweep at the merge.
    for k in (10, 12, 13, 14, 15, 16):
        out[f"roundabout_dropout_{k}"] = roundabout((float(k),), noise=faults)
    # The same with the detector not ready 0.5 s in every 2.5 s (empty list, ready False).
    windows = tuple((a, a + .5) for a in np.arange(6., 40., 2.5))
    for k in (12, 14):
        out[f"roundabout_not_ready_{k}"] = roundabout((float(k),), noise=faults, not_ready=windows)
    # A circulating car crawling (2 m/s, it slowed for us) that drives on at 12 km/h
    # 1 s later - its measured speed understates how soon it reaches the merge.
    for k in (12, 14):
        out[f"roundabout_resume_{k}"] = roundabout((float(k),), kmh=7., noise=faults, resume=(float(k) - 5., 12.))
    # Steady circulating traffic every 5 s, each car with a dropout: must still get in.
    out["roundabout_platoon_dropout"] = roundabout((9., 14., 19., 24., 29.), noise=faults, duration=70.)
    # Run 228: a car merges just ahead of us, then brakes to a stop in the merged lane
    # for 3 s (NPC yielding further on); we must follow it, never run into it.
    out["roundabout_lead_brake"] = roundabout((10.5, 30.), noise=faults, profile=[(14., 0.), (17., 12.)])
    # Run 222 t=256.7 replay: stopped 7 m past the line, nose at the circulating lane; the
    # car 10 m to our left (2 m/s, then 3.3 m/s) vanished 0.3 s after we accepted the gap,
    # came back 1.1 s later under a new id with no velocity, and ended beside our left front.
    out["roundabout_222_replay"] = roundabout((0.,), kmh=7., start_pose=(-98.0, 321.4, 67.9),
        noise=dict(drop_within_m=12., drop_s=1.15, drops=1, vok_delay_s=.55, initial_vok_s=0., drop_after_s=.3),
        resume=(1.0, 12.), start_xy=(-106.5, 326.3), duration=40.)

    # Tunnel: route idx at the tunnel entry anchor.
    tm_entry = np.array([28.42107705698054, -488.4178904363803])
    t0 = int(np.argmin(np.linalg.norm(world["smooth"] - tm_entry, axis=1))) - 30
    out["tunnel_empty"] = lambda: Sim(world, t0, speed_kmh=30., end_idx=t0 + 260, duration_s=60., stop_line=False, highway=False)
    out["tunnel_barrels"] = lambda: Sim(world, t0, speed_kmh=30., end_idx=t0 + 260, duration_s=70., stop_line=False, highway=False,
        actors=[static(world, t0, "Barrel", 50., 0., .6, .6, .9), static(world, t0, "Barrel", 85., 2., .6, .6, .9),
                static(world, t0, "CargoBox", 120., -1., 1.2, 1.2, 1.)])

    # Auto-collection run 49 (2026-10-04): studio "workzone" right at the tunnel mouth; the car
    # stopped against the concrete barrier. World xy from the generated scenario.
    def at(name, x, y, length, width, height=1.):
        k = int(np.argmin(np.linalg.norm(world["smooth"] - np.array([x, y]), axis=1)))
        d = world["smooth"][min(k + 1, len(world["smooth"]) - 1)] - world["smooth"][max(k - 1, 0)]
        return Actor(name, "obstacle", (x, y), math.degrees(math.atan2(d[1], d[0])), length, width, height)
    out["tunnel_workzone_mouth"] = lambda: Sim(world, t0 - 40, speed_kmh=30., end_idx=t0 + 260, duration_s=90.,
        stop_line=False, highway=False,
        actors=[at("ConcreteBarrier", 15.8, -497.0, 3.6, .6, .9), at("PEDrum", 10.1, -500.1, .6, .6, .9),
                at("Bollard", 6.1, -502.2, .3, .3, 1.), at("TrafficDrum", -.8, -506.0, .7, .7, 1.),
                at("RedDrum", -20.6, -516.4, .6, .6, .9), at("SteelBarrier", -26.0, -519.3, 3.0, .5, .9),
                at("YellowDrum", -32.4, -522.8, .6, .6, .9), at("LaneDivider", -37.2, -525.4, 3.6, .4, .9),
                at("TrafficDrum", -48.0, -529.1, .7, .7, 1.), at("TrafficDrum", -56.6, -534.4, .7, .7, 1.),
                at("RedDrum", -66.7, -540.1, .6, .6, .9)])
    # Auto run 111: a scooter right at the tunnel mouth and a parked Kona inside; stalled
    # at the mouth for 50 s in the odometry fallback (wall match unavailable).
    out["tunnel_mouth_scooter"] = lambda: Sim(world, t0 - 40, speed_kmh=30., end_idx=t0 + 200, duration_s=90.,
        stop_line=False, highway=False,
        actors=[at("Scooter", 18.0, -494.5, 1.8, .7, 1.2), at("KonaEV", 4.0, -502.8, 4.2, 1.8, 1.6)])
    # Auto run 118: a mixer truck (9 m x 2.5 m) parked skewed in the tunnel after a
    # scooter; "tunnel no safe bypass" 3.6 m behind it, three reverses, stuck.
    out["tunnel_mixer_truck"] = lambda: Sim(world, t0 + 10, speed_kmh=30., end_idx=t0 + 230, duration_s=90.,
        stop_line=False, highway=False,
        actors=[at("Scooter", -25.0, -518.8, 1.8, .7, 1.2),
                Actor("Mixer", "obstacle", (-36.0, -524.5), -142.3, 9.2, 2.5, 3.5)])
    # Auto run 158: drum slalom down the tunnel; the last drum sits 7 m before the exit.
    # Rejoining the route centre for the exit turned the path into it 3 m ahead at
    # 17 km/h (contact at 12 km/h).
    out["tunnel_exit_drum"] = lambda: Sim(world, t0 - 40, speed_kmh=30., end_idx=t0 + 260, duration_s=90.,
        stop_line=False, highway=False,
        actors=[at("KonaEV", 12.5, -497.1, 4.2, 1.8, 1.6), at("Kickboard", -21.2, -515.4, 1.4, .6, 1.1),
                at("Bollard", -43.8, -526.6, .3, .3, 1.), at("PEDrum", -50.6, -532.5, .6, .6, .9),
                at("YellowDrum", -58.5, -535.2, .6, .6, .9), at("TrafficDrum", -67.8, -542.1, .7, .7, 1.)])
    # S-shaped tunnel slalom (user 2026-10-05): drums alternating 0.6-1.4 m left/right
    # of the route every 10-16 m, as auto_collect now places them.
    def tunnel_frame(a, lat):
        import json as _j
        tm = _j.loads((HERE / "tunnel_wall_map.json").read_text())
        h = math.radians(tm["axis_heading_deg"]); o = np.asarray(tm["origin_xy"], float)
        return o + a * np.array([math.cos(h), math.sin(h)]) + lat * np.array([-math.sin(h), math.cos(h)])
    def s_slalom(seed):
        r = np.random.default_rng(seed)
        acts, a, side = [], 20. + r.uniform(0, 10), r.choice([-1., 1.])
        while a < 105.:
            x, y = tunnel_frame(a, side * r.uniform(.6, 1.4))
            acts.append(at(r.choice(["PEDrum", "TrafficDrum", "YellowDrum"]), float(x), float(y), .7, .7, 1.))
            a += r.uniform(10., 16.); side = -side
        return Sim(world, t0 - 30, speed_kmh=30., end_idx=t0 + 260, duration_s=90., stop_line=False,
                   highway=False, actors=acts)
    for k in range(3):
        out[f"tunnel_s_slalom_{k}"] = (lambda k=k: s_slalom(k))
    # Auto runs 185/187/188: tunnel exit, nothing on the road. The walls stand on a
    # 0.5 m kerb (survey kerb_face; the background filter drops it from the obstacle
    # points, only the LiDAR coverage sees it). Run 188 reached the exit 12 deg off
    # the axis heading for the right kerb; coverage read 5.5 m, the planner flipped
    # to "TUNNEL BYPASS" (+1.0 -> +2.5 m) and the car ended against the right kerb.
    # The bypass was triggered by returns just beyond the exit at the right wall line
    # (the route bends right outside); the sim walls run 5 m past the exit likewise.
    import json as _json
    tm_payload = _json.loads((HERE / "tunnel_wall_map.json").read_text())
    tm_heading, tm_end = float(tm_payload["axis_heading_deg"]), float(tm_payload["longitudinal_max_m"])
    exit_idx = int(np.argmin(np.linalg.norm(world["smooth"] - tunnel_frame(tm_end, -.9), axis=1)))
    def exit_kerb():
        a0, a1 = 40., tm_end
        xy = tunnel_frame((a0 + a1) / 2., tm_payload["right_offset_m"] + .25)
        return Actor("RightKerb", "obstacle", xy, tm_heading, a1 - a0, .5, 1.)
    def exit_sim(along, lat, yaw_error_deg, kmh, actors=(), exit_open=False):
        x, y = tunnel_frame(along, lat)
        return Sim(world, exit_idx - 60, speed_kmh=kmh, end_idx=exit_idx + 40, duration_s=40., stop_line=False,
                   highway=False, start_pose=(x, y, tm_heading + yaw_error_deg), exit_open=exit_open,
                   actors=[exit_kerb()] + list(actors))
    # Run 188 replay: 37 m before the exit, 3.9 m left of the route (after the slalom),
    # heading along the tunnel; the join to the route swings toward the right kerb.
    out["tunnel_exit_skewed_kerb"] = lambda: exit_sim(80., 3.2, .8, 27.)
    # Auto run 196: a 0.8 m concrete barrier (studio radius 1.8 -> 3.6 m) ~25 m into the
    # tunnel. Seen from 20 m, it sank into the VLP16 near blind zone once the car stopped
    # ~2 m short; 1.5 s later the near-field memory forgot it and the car drove into it
    # (61 s at full throttle, "stuck"). vfov models the blind zone. The recorded returns
    # show a face across the road (lat 0.3..3.35 m at along ~27.2); the scenario object
    # list gives yaw -152 deg (along the road) at along 25.0, lat 1.47. Both layouts:
    def low_barrier(along, lat, yaw_offset_deg):
        x, y = tunnel_frame(along, lat)
        return Actor("ConcreteBarrierC", "obstacle", (x, y), tm_heading + yaw_offset_deg, 3.6, .6, .8)
    out["tunnel_low_barrier_across"] = lambda: Sim(world, t0 - 30, speed_kmh=30., end_idx=t0 + 200, duration_s=90.,
        stop_line=False, highway=False, vfov=True, actors=[low_barrier(27.5, 1.8, 90.)])
    # The 196 stop: standing 5 m (vehicle origin) in front of the barrier face, only its
    # top edge in view. (196 stood nose 20 deg left; started skewed, the reverse escape
    # turns the car past the 35 deg wall-match limit -- a separate fallback stall.)
    def barrier_stop(lat0, yaw0):
        x, y = tunnel_frame(27.2 - 5.0, lat0)
        return Sim(world, t0, speed_kmh=0., end_idx=t0 + 200, duration_s=60., stop_line=False,
                   highway=False, vfov=True, start_pose=(x, y, tm_heading + yaw0),
                   actors=[low_barrier(27.5, 1.8, 90.)])
    out["tunnel_low_barrier_stopped"] = lambda: barrier_stop(1.8, 0.)
    out["tunnel_low_barrier_along"] = lambda: Sim(world, t0 - 30, speed_kmh=30., end_idx=t0 + 200, duration_s=90.,
        stop_line=False, highway=False, vfov=True, actors=[low_barrier(25., 1.5, 0.)])
    # Auto run 202: a +5 m bypass held to 28.5 m before the exit, nothing left on the
    # road. Entering the exit zone the centre return took a 6-8 m join (0.18 1/m):
    # heading -33 deg, 4.4 -> 0.9 m in 10 m, stopped 7 s at the right wall.
    out["tunnel_exit_offset_return"] = lambda: exit_sim(88., 4.5, 0., 29.)
    # Auto run 222: mid left bypass (+4 m) of an S row (drum at along 84, parked car
    # behind it) at 3 Hz decisions; the wall match failed twice ("odometry fallback"),
    # the car swung back toward the row and hit the drum at 14.8 km/h.
    def s_fallback(decision_s, fails):
        x, y = tunnel_frame(40., 4.)
        return Sim(world, t0, speed_kmh=30., end_idx=exit_idx + 40, duration_s=60., stop_line=False,
                   highway=False, start_pose=(x, y, tm_heading), decision_s=decision_s, match_fail=fails,
                   actors=[Actor("TrafficDrum", "obstacle", tunnel_frame(84., 1.2), tm_heading, .7, .7, 1.),
                           Actor("ParkedCar", "obstacle", tunnel_frame(89.5, .9), tm_heading, 4.2, 1.9, 1.5)])
    # Auto runs 271/225/222: the global_hard_09 tunnel S (scenario file positions; drums
    # alternating ~1 m either side every 10-13 m). 271: target +3.5 -> +0.5 crossed to the
    # right of the next drum (30004) 14 m ahead at 39 km/h, a 0.74 s decision gap, then
    # +5.0; hit at 14 km/h. steer_tau_s models the MPC trailing the path.
    def s_hard09(tau, kmh=38., wall_shift=0., start=None):
        drums = ((24.9, 1.20), (36.9, -.49), (50.1, .74), (60.4, -.97), (70.8, 1.09))
        pose = None if start is None else (*tunnel_frame(*start), tm_heading)
        return Sim(world, t0 - 30, speed_kmh=kmh, end_idx=exit_idx + 40, duration_s=60., stop_line=False,
                   highway=False, steer_tau_s=tau, wall_shift_m=wall_shift, start_pose=pose,
                   actors=[Actor("SDrum", "obstacle", tunnel_frame(a, l), tm_heading, .7, .7, 1.) for a, l in drums])
    out["tunnel_s_hard09"] = lambda: s_hard09(None)
    out["tunnel_s_hard09_lag"] = lambda: s_hard09(.4)
    out["tunnel_s_hard09_lag6"] = lambda: s_hard09(.6)
    out["tunnel_s_hard09_bias"] = lambda: s_hard09(.4, wall_shift=-.7)
    # 271 state at its first tunnel frame: +3.5 m bypass, 30 km/h (truth along 28.4, lat 3.43).
    out["tunnel_s_hard09_271"] = lambda: s_hard09(.4, kmh=30., wall_shift=-.7, start=(28.5, 3.4))
    out["tunnel_s_fallback_lowrate"] = lambda: s_fallback(.3, ((2.5, 3.1), (4.3, 5.0)))
    out["tunnel_s_fallback_long"] = lambda: s_fallback(.3, ((2.5, 6.5),))
    # The wall match fails mid-swerve around a drum on the route (3 Hz decisions):
    # the fallback must keep the swerve, not settle parallel at the half-way offset.
    def swerve_fallback(fail_from):
        x, y = tunnel_frame(40., -.7)
        return Sim(world, t0, speed_kmh=30., end_idx=exit_idx + 40, duration_s=40., stop_line=False,
                   highway=False, start_pose=(x, y, tm_heading), decision_s=.3,
                   match_fail=((fail_from, fail_from + 3.),),
                   actors=[Actor("TrafficDrum", "obstacle", tunnel_frame(62., -.7), tm_heading, .7, .7, 1.)])
    for k, fail_from in enumerate((.6, .9, 1.2)):
        out[f"tunnel_swerve_fallback_{k}"] = (lambda f=fail_from: swerve_fallback(f))
    # Open exit (no returns beyond it): the coverage is short, nothing on the road.
    out["tunnel_exit_open_coverage"] = lambda: exit_sim(80., -.7, 0., 29., exit_open=True)
    # Same open exit with a real drum on the route 10 m before it: must still avoid.
    out["tunnel_exit_open_drum"] = lambda: exit_sim(80., -.7, 0., 29., exit_open=True,
        actors=[Actor("TrafficDrum", "obstacle", tunnel_frame(tm_end - 10., -.8), tm_heading, .7, .7, 1.)])
    # Auto run 58: slalom bollard then a parked K5 in our lane; the left lane is free
    # (S-curve needed). Ended 3 m behind the K5.
    out["tunnel_slalom_parked"] = lambda: Sim(world, t0 + 10, speed_kmh=30., end_idx=t0 + 220, duration_s=90.,
        stop_line=False, highway=False,
        actors=[at("YellowDrum", -7.7, -507.8, .6, .6, .9), at("Bollard", -13.7, -513.2, .3, .3, 1.),
                at("ParkedK5", -23.7, -517.8, 4.9, 1.9, 1.5), at("RedDrum", -36.4, -523.1, .6, .6, .9)])
    # Auto run 77: parked cars alternating lanes; the fallback pulled the bypassing
    # car back into the second one.
    out["tunnel_parked_alternating"] = lambda: Sim(world, t0 + 10, speed_kmh=30., end_idx=t0 + 230, duration_s=90.,
        stop_line=False, highway=False,
        actors=[at("ParkedIoniq", 9.1, -500.0, 4.6, 1.9, 1.6), at("ParkedIoniq", -6.7, -507.7, 4.6, 1.9, 1.6),
                at("ParkedGrandeur", -22.2, -515.9, 5.0, 1.9, 1.5)])
    # Scenario Studio box-section layouts (auto_collect "box" mode), static actors only.
    studio_file = HERE / "sim_studio_layouts.json"
    if studio_file.exists():
        sizes = {"barrel": None, "box": None, "vehicle": None, "barrier": None, "bike": (1.8, .6, 1.1)}
        for k, layout in enumerate(json.loads(studio_file.read_text())):
            def build(layout=layout):
                acts = []
                for a in layout["actors"]:
                    cat, r = a.get("cat"), float(a.get("r", .5))
                    length, width, height = sizes.get(cat) or (
                        (2 * r, .6, .9) if cat == "barrier" else
                        (2 * r, 2.5 if r > 4 else 1.9, 2.5 if r > 3 else 1.5) if cat == "vehicle" else (2 * r, 2 * r, .9))
                    acts.append(Actor(a.get("model") or cat, "obstacle", (a["x"], a["y"]), float(a.get("yaw", 0.)),
                                      length, width, height))
                return Sim(world, 470, speed_kmh=40., end_idx=1100, duration_s=150., stop_line=False, highway=False,
                           actors=acts)
            out[f"studio_box_{k}"] = build
    # Stopped 7-9 m behind blockers inside the tunnel: the escape must steer out
    # (runs 52/57 parked for good). Starts at 0 km/h.
    for k, (lat, gap) in enumerate(((0., 7.), (.6, 8.), (-.5, 9.))):
        out[f"tunnel_stalled_escape_{k}"] = (lambda lat=lat, gap=gap: Sim(world, t0 + 60, speed_kmh=0., end_idx=t0 + 200,
            duration_s=60., stop_line=False, highway=False,
            actors=[static(world, t0 + 60, "ParkedCar", gap + 2.3 + 2.2, lat, 4.4, 1.8, 1.6)]))
    # Auto run 52: parked cars in the tunnel ("bottleneck"/"parked"); stuck 60 s behind the Niro.
    out["tunnel_parked_bottleneck"] = lambda: Sim(world, t0 + 40, speed_kmh=30., end_idx=t0 + 260, duration_s=90.,
        stop_line=False, highway=False,
        actors=[at("ParkedK5", -11.1, -511.0, 4.9, 1.9, 1.5), at("ParkedNiro", -28.0, -519.2, 4.4, 1.8, 1.6),
                at("ParkedK5", -46.9, -529.2, 4.9, 1.9, 1.5)])

    # Guardrail S-curve (route idx 1270-1720, zone 55 km/h, 45 elsewhere): from the
    # junction exit (28 km/h as in runs 180-197) to the straight before the next
    # junction. Run with SIM_CURVE_FOLLOWER=1 for the follower's curve speed.
    zone = float(os.environ.get("SIM_CURVE_ZONE_KMH", "58"))
    def curve(actors=(), start=1280, kmh=28., guardrail=False, **sim_options):
        def build():
            acts = list(actors() if callable(actors) else actors)
            if guardrail:
                acts += guardrail_boxes(guardrail if callable(guardrail) else None)
            sim = Sim(world, start, speed_kmh=kmh, end_idx=1715, duration_s=60., stop_line=False, highway=False,
                      actors=acts, drift=guardrail, model_points=12, **sim_options)
            sim.rt.r.speed_zones = SimpleNamespace(
                kmh_at=lambda index: ((zone, {"kmh": zone}) if 1270 <= index <= 1720 else (45., {"kmh": 45.})))
            return sim
        return build
    # Median guardrail where LiDAR measured its face (racing_line.json guard_m: runs
    # 190/193/200/202, conservative), 0.8 m high, 4 m pieces; LiDAR only (fusion never
    # lists it). Used with the measured MPC drift toward it (run 203 touched it).
    def guardrail_boxes(face=None):
        payload = json.loads((HERE / "racing_line.json").read_text())
        first, guard = int(payload["first_idx"]), payload["guard_m"]
        out_boxes = []
        for k in range(0, len(guard), 8):
            xy, h = route_frame(world, first + k, 0., (float(guard[k]) if face is None else face(first + k)) + .15)
            box = Actor("Guardrail", "obstacle", xy, h, 4.2, .3, .8)
            box.lidar_only = True
            out_boxes.append(box)
        return out_boxes
    out["curve_empty"] = curve()
    # A car in the right lane ahead (30 km/h): the racing line must not be used near it.
    out["curve_car_right_lane"] = curve(lambda: [Actor("right_car", "npc", route_frame(world, 1280, 70., -3.5)[0], 0.,
        4.6, 1.9, 1.5, speed=30 / 3.6, path=route_polyline(world, 1280, 70., 400., -3.5), uid=960)])
    # A faster car closing from behind in the right lane (70 km/h).
    out["curve_car_behind_right"] = curve(lambda: [Actor("behind_car", "npc", route_frame(world, 1280, -25., -3.5)[0], 0.,
        4.6, 1.9, 1.5, speed=70 / 3.6, path=route_polyline(world, 1280, -25., 400., -3.5), uid=961)])
    # A stopped car in the right lane at the apex of the racing line (idx ~1550).
    out["curve_stopped_right_apex"] = curve(lambda: [static(world, 1280, "StoppedCar", 135., -3.4, 4.6, 1.9, 1.5)])
    # Cargo box on the right-lane edge where the racing line crosses the lanes (idx ~1505).
    out["curve_box_lane_mark"] = curve(lambda: [static(world, 1280, "CargoBox", 112., -2.6, 1.2, 1.2, 1.0)])
    # Run 203: guardrail + measured drift, empty road / car alongside in lane 2 /
    # slow car ahead in lane 1 (racing off: the guardrail line).
    out["curve_guardrail"] = curve(guardrail=True)
    out["curve_guardrail_car_lane2"] = curve(lambda: [Actor("lane2_car", "npc", route_frame(world, 1280, 5., -3.5)[0], 0.,
        4.6, 1.9, 1.5, speed=50 / 3.6, path=route_polyline(world, 1280, 5., 400., -3.5), uid=962)], guardrail=True)
    # Run 227: no vehicle anywhere, but (1) fusion listed the median guardrail as
    # 3 x 1.1 m "fusion_obstacle" boxes 0.3-0.8 m left of the route at idx 1510-1532
    # (and once at -0.8 m, idx 1552) while the car was at idx 1436-1476: racing gate off
    # + 2 s hold; (2) the ESKF heading was 2 deg clockwise of truth in the right bend and
    # 4-5 deg at idx 1475-1515, so far LiDAR returns of the guardrail fell on the guardrail
    # line ("lane 2: lidar 28-45 m", safe_lane/safe_wide every other frame) and the car
    # drifted left of each re-anchored path; the racing line came back at idx 1504 with
    # the car 3 m left of it and the car reached +0.96 m (LiDAR body gap 0.05 m), then the
    # control-path LiDAR check stopped it 38 -> 1 km/h at idx 1530-1540.
    def run227_bias(idx):
        ramp = lambda a, b: float(np.clip((idx - a) / float(b - a), 0., 1.))
        base = -2.2 * ramp(1330, 1350) * (1. - ramp(1530, 1560))
        extra = -2.6 * ramp(1465, 1485) * (1. - ramp(1505, 1525))
        return base + extra
    run227_fp = [(1515, .47, 2.95), (1523, .30, 2.94), (1510, .75, 2.97), (1518, .49, 2.97),
                 (1523, .41, 2.94), (1518, .50, 2.98), (1552, -.81, 1.94), (1532, -.25, 1.65)]
    def run227_ghosts():
        ghosts = []
        for k, (idx, lat, length) in enumerate(run227_fp):
            box = static(world, idx, "fusion_obstacle", 0., lat, length, 1.1, 1.)
            box.ghost = (lambda i, frame, k=k: 1436 <= i <= 1476 and frame % len(run227_fp) == k)
            ghosts.append(box)
        return ghosts
    # Guardrail face where run 227's LiDAR saw it with the MORAI truth pose (idx 1400-1590:
    # 1.9-2.06 m, 2.3-2.4 m past idx 1600; guard_m is 0.25-0.55 m more conservative).
    run227_face = lambda idx: float(np.interp(idx, [1280, 1400, 1590, 1605, 1700], [2.0, 2.0, 2.05, 2.35, 2.4]))
    out["curve_guardrail_227"] = curve(run227_ghosts, guardrail=run227_face, heading_bias=run227_bias)
    out["curve_guardrail_slow_lane1"] = curve(lambda: [Actor("lane1_car", "npc", route_frame(world, 1280, 60., 0.)[0], 0.,
        4.6, 1.9, 1.5, speed=35 / 3.6, path=route_polyline(world, 1280, 60., 400., 0.), uid=963)], guardrail=True)
    return out


def crossing_link(world, stop_idx, link_id="A2256W000134"):
    """Traffic merging into our right turn: the map link from the left,
    extended 60 m upstream, then our own route downstream of the merge."""
    import ast
    lanes = world["lanes"]
    link = next((l for l in lanes.links if link_id in (l.get("idx"), l.get("link_id"), l.get("id"))), None)
    if link is None:
        raise RuntimeError("merge link not found: " + link_id)
    raw = link["points"]
    pts = np.asarray(ast.literal_eval(raw) if isinstance(raw, str) else raw, float)[:, :2]
    direction = (pts[1] - pts[0]) / np.linalg.norm(pts[1] - pts[0])
    upstream = pts[0] - direction * np.arange(60., 0., -1.)[:, None]
    smooth = world["smooth"]
    merge = int(np.argmin(np.linalg.norm(smooth[stop_idx:stop_idx + 120] - pts[-1], axis=1))) + stop_idx
    return np.vstack((upstream, pts, smooth[merge + 1: merge + 300]))


def random_box(world, seed, start=480):
    """Unknown competition placement: 2-5 random obstacles in the box section."""
    rng = np.random.default_rng(seed)
    kinds = [("CargoBox", 1.2, 1.2, 1.0), ("Barrel", .6, .6, .9), ("Bollard", .3, .3, 1.0),
             ("TrafficBarrel", .7, .7, 1.0), ("ParkedCar", 4.6, 1.9, 1.5)]
    actors, along = [], 30.
    for _ in range(int(rng.integers(2, 6))):
        along += float(rng.uniform(15., 35.))
        if along > 150.:
            break
        name, length, width, height = kinds[int(rng.integers(len(kinds)))]
        lateral = float(rng.uniform(-1.6, 1.6))
        if name == "ParkedCar" and abs(lateral) < 1.:
            lateral = float(np.sign(lateral or 1.)) * 1.2
        actors.append(static(world, start, name, along, lateral, length, width, height))
    return Sim(world, start, speed_kmh=40., end_idx=start + 380, duration_s=70., stop_line=False,
               highway=False, actors=actors)


def main(names):
    world = load_world()
    table = scenarios(world)
    selected = names or list(table)
    results = {}
    for name in selected:
        started = time.perf_counter()
        sim = table[name]()
        stats = sim.run()
        stats["wall_s"] = round(time.perf_counter() - started, 1)
        results[name] = stats
        flag = "COLLISION" if stats["collision"] else ("OK" if stats["reached"] else "NOT REACHED")
        print(f"[{flag}] {name}: t={stats['t']}s dist={stats['distance']}m mean={stats['mean_kmh']}km/h "
              f"stopped={stats['stop_s']}s (longest {stats['longest_stop_s']}s) min_gap={stats['min_gap']}m",
              flush=True)
        if stats["collision"]:
            print("    collision:", stats["collision"])
        if stats["stop_reasons"]:
            print("    stops:", stats["stop_reasons"])
        print("    modes:", stats["modes"])
        out = Path(os.environ.get("SIM_LOG_DIR", tempfile.gettempdir())) / f"sim_{name}.jsonl"
        out.write_text("\n".join(json.dumps(row) for row in sim.log))
        (out.parent / f"stats_{name}.json").write_text(json.dumps(stats, ensure_ascii=False))
    return results


if __name__ == "__main__":
    main(sys.argv[1:])
