#!/usr/bin/env python3
"""Detect and track cars on the highway, intersections and tunnel approach."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "autodrive"))
# STUDENT_V3_DIR: the launch exports its code copy; the default is the E2E copy.
sys.path.insert(0, os.environ.get("STUDENT_V3_DIR", str(ROOT / "autodrive/student_v3_claude")))
from motion_filter import measured_velocity
from perception.roadside_structures import RoadsideStructures
from intersection_perception import active_intersection, load_intersection_regions, DETECTOR_PADDING_M


def detector_mission_active(index, mission, intersection_regions=(), route_station=None):
    """Keep the detector ready across independently sampled mission boundaries.

    The GUI can still be in the final highway frame while this subscriber's
    newer pose has passed the end. Retain inference for another 100 route
    points (about 50 m), covering the pose and decision pipeline handoff.
    """
    if mission["from_idx"]-200 <= index <= mission["to_idx"]+100:
        return True
    if any(region['from_idx'] <= index <= region['to_idx'] for region in mission.get('lidar_regions', ())):
        return True
    if not intersection_regions:
        return False
    if route_station is None:
        raise ValueError("Intersection perception requires route station geometry")
    return (0 <= index < len(route_station) and active_intersection(
        route_station[index], intersection_regions, DETECTOR_PADDING_M) is not None)


def cloud_xyzi(message):
    types = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
             5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}
    fields = {f.name: f for f in message.fields}
    if not {"x", "y", "z", "intensity"}.issubset(fields):
        raise ValueError("VLP16 requires x/y/z/intensity fields")
    dtype = np.dtype({"names": list(fields), "formats": [types[f.datatype] for f in fields.values()],
                      "offsets": [f.offset for f in fields.values()], "itemsize": message.point_step})
    data = np.frombuffer(message.data, dtype=dtype, count=message.width*message.height)
    result = np.column_stack([data[name] for name in ("x", "y", "z", "intensity")]).astype(np.float32)
    return result[np.isfinite(result).all(axis=1)]



TRACK_KEEP_S = 1.0   # 0.5: at the 5 Hz detector rate one missed frame at 40 m restarted the
                     # track, so a car ahead never got a valid speed and read as static (run 128)
CENTROID_LATERAL_NOISE_M = .3
TRACK_LATERAL_ACCEL_MPS2 = 6.


def vehicle_velocity(history):
    """measured_velocity over 1.2 s with >= 3 samples (detector ~5 Hz with misses)."""
    samples = list(history)
    if len(samples) < 3:
        return None
    recent = np.asarray([s for s in samples if samples[-1][0] - s[0] <= 1.2], dtype=float)
    if len(recent) < 3 or recent[-1, 0] - recent[0, 0] < 0.3 - 1e-9:
        return None
    dt = recent[None, :, 0] - recent[:, None, 0]
    pairs = dt >= 0.3 - 1e-9
    slopes = (recent[None, :, 1:3] - recent[:, None, 1:3])[pairs] / dt[pairs, None]
    velocity = np.median(slopes, axis=0)
    if not np.isfinite(velocity).all() or np.linalg.norm(velocity) > 45.:
        return None
    elapsed = recent[:, 0] - recent[-1, 0]
    intercepts = recent[:, 1:3] - elapsed[:, None] * velocity
    residual = float(np.median(np.linalg.norm(intercepts - np.median(intercepts, axis=0), axis=1)))
    displacement = float(np.linalg.norm(velocity) * (recent[-1, 0] - recent[0, 0]))
    if displacement < max(0.15, 3.0 * residual):
        return 0.0, 0.0
    return float(velocity[0]), float(velocity[1])

class VehicleTracks:
    """Stable IDs and world velocities; pose-compensated measurements only."""
    def __init__(self):
        self.tracks = {}
        self.next_id = 500000

    def reclassify_roadside(self, structures, detections):
        """Retire a proven false car instead of forecasting it for another second."""
        removed = []
        for uid, track in list(self.tracks.items()):
            position = np.asarray(track['position'], float)
            if any(np.linalg.norm(position-np.asarray(d['position'])) <= .5 for d in detections):
                continue
            for structure in structures:
                if np.linalg.norm(position-np.asarray(structure['position'])) > .5:
                    continue
                angle = abs((track['heading_deg']-structure['heading_deg']+90.) % 180.-90.)
                if angle <= 20. and np.all(np.abs(np.asarray(track['size'])-structure['size']) <=
                                          .25*np.asarray(track['size'])):
                    removed.append(uid)
                    del self.tracks[uid]
                    break
        return removed

    def update(self, detections, stamp, min_score=None):
        if any(stamp < track["stamp"] for track in self.tracks.values()):
            self.tracks.clear()
        previous = {key: value for key, value in self.tracks.items() if 0 <= stamp-value["stamp"] <= TRACK_KEEP_S}
        used, output = set(), []
        # High-confidence observations own their matches before a weak return
        # can continue a track seeded by a high-confidence detection. Its
        # velocity remains unconfirmed until enough displacement is measured.
        ordered = sorted(detections, key=lambda d: -float(d.get("score", 1.)))
        for detection in ordered:
            weak = min_score is not None and float(detection.get("score", 0.)) < min_score
            if weak and float(detection.get("score", 0.)) < .2:
                continue
            position = np.asarray(detection["position"])
            best, distance = None, 2. if weak else 4.5
            for key, track in previous.items():
                if key in used:
                    continue
                if weak:
                    size = np.asarray(detection["size"])
                    previous_size = np.asarray(track["size"])
                    if np.any(np.abs(size-previous_size) > .25*previous_size):
                        continue
                    turn = abs((detection["heading_deg"]-track["heading_deg"]+90.) % 180.-90.)
                    if turn > 20.:
                        continue
                expected = np.asarray(track["position"])+np.asarray(track["velocity"])*(stamp-track["stamp"])
                gap = float(np.linalg.norm(position-expected))
                if gap < distance:
                    best, distance = key, gap
            if best is None:
                if weak:
                    continue
                best, self.next_id = self.next_id, self.next_id+1
                history = deque(maxlen=32)
            else:
                track = previous[best]
                history = track["history"]
                dt = stamp-track['stamp']
                motion = np.asarray(track['velocity'], dtype=float)
                speed = float(np.linalg.norm(motion))
                measured = track['velocity_valid'] and speed > 2.
                if not measured and len(history) >= 2:
                    baseline = history[-1][0]-history[-2][0]
                    if baseline > 0.:
                        motion = (np.asarray(history[-1][1:3])-history[-2][1:3])/baseline
                        speed = float(np.linalg.norm(motion))
                if dt > 0. and speed > 2.:
                    expected = np.asarray(track['position'])+motion*dt
                    normal = np.array([-motion[1], motion[0]])/speed
                    lateral_offset = float((position-expected) @ normal)
                    lateral_jump = abs(lateral_offset)
                    # Partial VLP16 boxes can move their centroid >1 m in one
                    # scan while the physical car continues straight. Keep the
                    # current measured footprint. For established motion,
                    # re-anchor the velocity history to the new centroid so
                    # neither sideways motion nor a sudden standstill is invented.
                    # Innovation is relative to the already predicted motion,
                    # so acceleration, rather than another lateral velocity,
                    # bounds a real change between scans. Re-anchor partial
                    # box offsets without moving the measured footprint.
                    noise = CENTROID_LATERAL_NOISE_M
                    allowance = noise + .5*TRACK_LATERAL_ACCEL_MPS2*dt*dt
                    shape_heading = float(detection['heading_deg'])
                    supported_turn = False
                    if measured and math.isfinite(shape_heading):
                        shape_yaw = math.radians(shape_heading)
                        shape_axis = np.array([math.cos(shape_yaw), math.sin(shape_yaw)])
                        if float(shape_axis @ motion) < 0.:
                            shape_axis *= -1.
                        shape_lateral = speed*float(shape_axis @ normal)
                        # A real turn can exceed the innovation bound because
                        # velocity fitting lags acceleration. Current box
                        # orientation independently supports that manoeuvre.
                        supported_turn = (shape_lateral*lateral_offset > 0. and
                            abs(shape_lateral)*dt >= lateral_jump-noise)
                    if lateral_jump > allowance and not supported_turn:
                        if measured:
                            correction = lateral_offset*normal
                            history = deque(((t, x+correction[0], y+correction[1])
                                             for t, x, y in history), maxlen=32)
                        else:
                            history = deque(maxlen=32)
            used.add(best)
            if not history or stamp > history[-1][0]:
                history.append((stamp, *position))
            velocity = vehicle_velocity(history)
            reliable = velocity is not None
            velocity = velocity or (0., 0.)
            motion_heading = (math.degrees(math.atan2(velocity[1], velocity[0]))
                              if reliable and math.hypot(*velocity) > .3 else None)
            item = dict(detection, uid=best, velocity=list(velocity), velocity_valid=reliable,
                        motion_heading_deg=motion_heading, stamp=stamp, history=history)
            self.tracks[best] = item
            output.append({k: v for k, v in item.items() if k != "history"})
        for key, track in previous.items():
            if key not in used and stamp-track["stamp"] <= TRACK_KEEP_S:
                item = {k: v for k, v in track.items() if k != "history"}
                age = stamp-track["stamp"]
                item["position"] = (np.asarray(track["position"])+np.asarray(track["velocity"])*age).tolist()
                # Retain a measured track through a missed scan; do not invent
                # motion for an unconfirmed track or refresh its observation age.
                item["prediction_age_s"] = age
                output.append(item)
        self.tracks = {key: value for key, value in self.tracks.items() if 0 <= stamp-value["stamp"] <= TRACK_KEEP_S}
        return output


class VehicleReceiver:
    """Read the dedicated detector topic without a ground-truth file fallback."""
    def __init__(self, topic="/perception/lidar_vehicles"):
        import rospy
        from std_msgs.msg import String
        self.lock = threading.Lock()
        self.record, self.received = {}, 0.
        self.subscriber = rospy.Subscriber(topic, String, self._on_message, queue_size=1)

    def _on_message(self, message):
        try:
            record = json.loads(message.data)
            if not isinstance(record.get("vehicles"), list):
                return
        except (ValueError, TypeError):
            return
        with self.lock:
            self.record, self.received = record, time.monotonic()

    def snapshot(self):
        with self.lock:
            record, age = self.record, time.monotonic()-self.received
        ready = age <= .35 and bool(record.get("ready")) and bool(record.get("active"))
        objects = []
        if ready:
            for car in record["vehicles"]:
                x, y = car["position"]
                vx, vy = car["velocity"]
                length, width = car["size"][:2]
                objects.append((x, y, max(length, width), "lidar_car", "npc", math.hypot(vx, vy),
                                length, width, car["heading_deg"], vx, vy, car["uid"], 0., None,
                                bool(car["velocity_valid"]), float(car.get("prediction_age_s", 0.))))
        return objects, ready, dict(record, age_s=round(age, 3))

    def close(self):
        self.subscriber.unregister()


def scan_pose(history, stamp):
    """Align localization with the scan using bounded measured motion.

    ESKF can arrive in bursts about 0.18 s behind LiDAR. A missing matching
    pose for the newest scan does not invalidate the preceding tracked scan.
    """
    before = [p for p in history if p[0] <= stamp]
    after = [p for p in history if p[0] >= stamp]
    if before and after:
        a, b = max(before, key=lambda p: p[0]), min(after, key=lambda p: p[0])
        dt = b[0]-a[0]
        if dt <= .35:
            u = (stamp-a[0])/dt if dt > 1e-6 else 0.
            turn = (b[3]-a[3]+math.pi) % (2*math.pi)-math.pi
            return (stamp, a[1]+u*(b[1]-a[1]), a[2]+u*(b[2]-a[2]), a[3]+u*turn)
    if before and not after:
        b = max(before, key=lambda p: p[0])
        delay = stamp-b[0]
        earlier = [p for p in before if .05 <= b[0]-p[0] <= .35]
        if delay <= .25 and earlier:
            a = min(earlier, key=lambda p: abs(b[0]-p[0]-.15))
            dt = b[0]-a[0]
            vx, vy = (b[1]-a[1])/dt, (b[2]-a[2])/dt
            yaw_rate = ((b[3]-a[3]+math.pi) % (2*math.pi)-math.pi)/dt
            if math.hypot(vx, vy) <= 45. and abs(yaw_rate) <= 1.:
                return (stamp, b[1]+vx*delay, b[2]+vy*delay, b[3]+yaw_rate*delay)
    pose = min(history, key=lambda p: abs(p[0]-stamp)) if history else None
    return pose if pose is not None and abs(pose[0]-stamp) <= .05 else None


def tracked_scan_expired(now, stamp, last_processed_stamp, max_age=.35):
    """Reject true input loss, allowing a fresh result while pose sync catches up."""
    return (last_processed_stamp is None or not -.05 <= now-stamp <= max_age or
            not -.05 <= now-last_processed_stamp <= max_age)


def aligned_scan(messages, history, now, last_stamp):
    """Use the newest scan whose matching ESKF pose has arrived."""
    for message in reversed(messages):
        stamp = message.header.stamp.to_sec()
        if last_stamp is not None and stamp <= last_stamp:
            continue
        if not -.05 <= now-stamp <= .35:
            continue
        pose = scan_pose(history, stamp)
        if pose is not None:
            return message, pose
    return None, None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--route", type=Path, required=True)
    parser.add_argument("--sensor", type=Path, required=True)
    parser.add_argument("--intersection-regions", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, default=ROOT / "trained_models/morai_pointpillar_20k_2026-08-14")
    parser.add_argument("--ready-file", type=Path)
    parser.add_argument("--hz", type=float, default=15.)
    parser.add_argument("--score-threshold", type=float, default=.445)
    parser.add_argument("--lidar-topic", default="/lidar/points")
    parser.add_argument("--ego-topic", default="/localization/ego_status")
    parser.add_argument("--out-topic", default="/perception/lidar_vehicles")
    args = parser.parse_args()
    if not math.isfinite(args.hz) or not 1 <= args.hz <= 15:
        parser.error("--hz must be finite and within 1–15 Hz")
    for directory in ("/opt/ros/noetic/lib/python3/dist-packages",
                      str(ROOT / "localization/devel/lib/python3/dist-packages"),
                      "/usr/lib/python3/dist-packages"):
        sys.path.append(directory)
    import rospy
    from morai_msgs.msg import EgoVehicleStatus
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from scipy.spatial import cKDTree
    import torch
    from pointpillar_runtime import PointPillarRuntime
    torch.set_num_threads(2)
    rospy.init_node("highway_lidar_vehicles", anonymous=False)
    mission = json.loads(args.mission.read_text())
    route = json.loads(args.route.read_text())["waypoints"]
    route_xy = np.asarray([[p["x"], p["y"]] for p in route], dtype=float)
    route_station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route_xy, axis=0), axis=1))]
    intersections = load_intersection_regions(args.intersection_regions, route_xy)
    tree = cKDTree(route_xy)
    roadside_map = RoadsideStructures(route_xy, map_path=args.route.parent/'link_set.json')
    sensor = json.loads(args.sensor.read_text())["Lidar3DList"][0]
    mount = np.array([float(sensor["pos"][k]) for k in ("x", "y", "z")])
    lock, clouds, poses = threading.Lock(), deque(maxlen=8), deque(maxlen=200)
    cloud_receipts = deque(maxlen=60)
    def on_cloud(message):
        with lock:
            clouds.append(message)
            cloud_receipts.append(time.monotonic())
    def on_ego(message):
        with lock:
            poses.append((message.header.stamp.to_sec(), message.position.x, message.position.y,
                          math.radians(message.heading)))
    rospy.Subscriber(args.lidar_topic, PointCloud2, on_cloud, queue_size=1, buff_size=2**24)
    rospy.Subscriber(args.ego_topic, EgoVehicleStatus, on_ego, queue_size=10)
    publisher = rospy.Publisher(args.out_topic, String, queue_size=1)
    runtime = PointPillarRuntime(args.model_dir/"pointpillar_morai.yaml", args.model_dir/"morai_dataset.yaml",
                                 args.model_dir/"best_model_epoch_20.pth", strict_checkpoint=True)
    if args.ready_file:
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = args.ready_file.with_suffix(".tmp")
        tmp.write_text(json.dumps(dict(runtime_ready=True, model=str(args.model_dir),
            target_hz=args.hz, device=torch.cuda.get_device_name(0), started=time.time(),
            intersection_regions=intersections)))
        tmp.replace(args.ready_file)
    print(json.dumps(dict(runtime_ready=True, model=str(args.model_dir), hz=args.hz,
        device=torch.cuda.get_device_name(0))), flush=True)
    tracker, last_stamp, active_frames = VehicleTracks(), None, 0
    processed, measured_hz, rate_start = 0, 0., time.monotonic()
    rate = rospy.Rate(args.hz)
    while not rospy.is_shutdown():
        with lock:
            messages, history = list(clouds), list(poses)
            input_times = list(cloud_receipts)
        input_hz = ((len(input_times)-1)/(input_times[-1]-input_times[0])
                    if len(input_times)>1 and input_times[-1]>input_times[0] else 0.)
        message = messages[-1] if messages else None
        active, ready, vehicles, inference_ms = False, False, [], None
        if message is not None and history:
            stamp = message.header.stamp.to_sec()
            latest_pose = history[-1]
            _, index = tree.query([latest_pose[1], latest_pose[2]])
            active = detector_mission_active(index, mission, intersections, route_station)
            now_ros = rospy.Time.now().to_sec()
            aligned, pose = aligned_scan(messages, history, now_ros, last_stamp)
            fresh = aligned is not None
            if active and fresh:
                message, stamp = aligned, aligned.header.stamp.to_sec()
                began = time.perf_counter()
                points = cloud_xyzi(message)
                body = points[:, :3]+mount
                # Own body/hood must not become the model's Car prediction.
                outside = (np.abs(body[:, 0]) > 2.3675) | (np.abs(body[:, 1]) > .996)
                points = points[outside]
                # Map-owned infrastructure is background for this detector,
                # rather than a car that must be removed after inference. The
                # raw ROS cloud used by collision checks is never modified.
                roadside_points = roadside_map.local_point_mask(points, pose, mount=mount)
                roadside_points_dropped = int(roadside_points.sum())
                points = points[~roadside_points]
                boxes, scores, _ = runtime.infer(points, stamp)
                torch.cuda.synchronize()
                inference_ms = (time.perf_counter()-began)*1000.
                detections, roadside_candidates = [], []
                c, s = math.cos(pose[3]), math.sin(pose[3])
                for box, score in zip(boxes, scores):
                    x, y, z, length, width, height, heading = box
                    x, y = x+mount[0], y+mount[1]
                    if not (.8 <= length <= 9. and .5 <= width <= 4.):
                        continue
                    if abs(x) <= 2.3675 and abs(y) <= .996:
                        continue
                    structure = roadside_map.vehicle_structure(points, box, mount, pose)
                    if structure is not None:
                        roadside_candidates.append(dict(roadside_structure=structure[0],
                            source_points=structure[1], score=float(score),
                            size=[float(length), float(width), float(height)],
                            heading_deg=math.degrees(float(heading)+pose[3]),
                            position=[float(pose[1]+c*x-s*y), float(pose[2]+s*x+c*y)]))
                        continue
                    detections.append(dict(position=[float(pose[1]+c*x-s*y), float(pose[2]+s*x+c*y)],
                                           local_position=[float(x), float(y)], local_z=float(z+mount[2]), score=float(score),
                                           size=[float(length), float(width), float(height)],
                                           heading_deg=math.degrees(float(heading)+pose[3])))
                roadside_retired_ids = tracker.reclassify_roadside(roadside_candidates, detections)
                vehicles = tracker.update(detections, stamp, min_score=args.score_threshold)
                processing_ms = (time.perf_counter()-began)*1000.
                active_frames += 1
                processed += 1
                now = time.monotonic()
                if now-rate_start >= 1.:
                    measured_hz, processed, rate_start = processed/(now-rate_start), 0, now
                ready, last_stamp = active_frames >= 2, stamp
                publisher.publish(String(data=json.dumps(dict(time=time.time(), stamp=stamp, active=active, runtime_ready=True,
                    ready=ready, target_hz=args.hz, hz=round(measured_hz, 1), inference_ms=round(inference_ms, 1),
                    input_scan_hz=round(input_hz, 1), processing_ms=round(processing_ms, 1),
                    source_age_s=round(now_ros-stamp, 3),
                    pose_delay_s=round(stamp-latest_pose[0], 3),
                    roadside_candidates=roadside_candidates, roadside_retired_ids=roadside_retired_ids,
                    roadside_points_dropped=roadside_points_dropped,
                    vehicles=vehicles), separators=(",", ":"))))
            elif not active or (not fresh and tracked_scan_expired(now_ros, stamp, last_stamp)):
                active_frames = 0
                tracker.tracks.clear()
                publisher.publish(String(data=json.dumps(dict(time=time.time(), active=active, ready=False, runtime_ready=True,
                    target_hz=args.hz, hz=0., vehicles=[]), separators=(",", ":"))))
        if message is None or not history:
            publisher.publish(String(data=json.dumps(dict(time=time.time(), active=False, ready=False,
                runtime_ready=True, target_hz=args.hz, hz=0., vehicles=[]))))
        rate.sleep()


if __name__ == "__main__":
    main()
