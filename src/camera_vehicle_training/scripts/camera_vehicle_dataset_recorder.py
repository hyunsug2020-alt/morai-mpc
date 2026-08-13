#!/usr/bin/env python3
"""Save MORAI front-camera images with interpolated automatic YOLO labels."""

import json
import math
import os
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rospy
from camera_vehicle_training.geometry import (
    make_vehicle_box_corners,
    projected_box,
    world_object_to_vehicle,
    xyxy_to_yolo,
)
from cv_bridge import CvBridge, CvBridgeError
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def interpolate_angle_degrees(first, second, ratio):
    delta = math.degrees(
        normalize_angle(math.radians(float(second) - float(first)))
    )
    return float(first) + float(ratio) * delta


def interpolation_ratio(first_stamp, second_stamp, target_stamp):
    interval = float(second_stamp) - float(first_stamp)
    if interval <= 1.0e-9:
        return 0.0
    return min(1.0, max(0.0, (target_stamp - first_stamp) / interval))


class VectorState:
    __slots__ = ("x", "y", "z")

    def __init__(self, x, y, z):
        self.x, self.y, self.z = float(x), float(y), float(z)

    @classmethod
    def from_message(cls, message):
        return cls(message.x, message.y, message.z)

    @classmethod
    def interpolate(cls, first, second, ratio):
        return cls(
            first.x + ratio * (second.x - first.x),
            first.y + ratio * (second.y - first.y),
            first.z + ratio * (second.z - first.z),
        )

    def as_list(self):
        return [self.x, self.y, self.z]


class EgoState:
    __slots__ = ("stamp", "position", "heading")

    def __init__(self, stamp, position, heading):
        self.stamp = float(stamp)
        self.position = position
        self.heading = float(heading)

    @classmethod
    def from_message(cls, message):
        return cls(
            message.header.stamp.to_sec(),
            VectorState.from_message(message.position),
            message.heading,
        )

    @classmethod
    def interpolate(cls, first, second, stamp):
        ratio = interpolation_ratio(first.stamp, second.stamp, stamp)
        return cls(
            stamp,
            VectorState.interpolate(first.position, second.position, ratio),
            interpolate_angle_degrees(first.heading, second.heading, ratio),
        )


class ObjectState:
    __slots__ = ("unique_id", "name", "position", "size", "heading")

    def __init__(self, unique_id, name, position, size, heading):
        self.unique_id = int(unique_id)
        self.name = str(name)
        self.position = position
        self.size = size
        self.heading = float(heading)

    @classmethod
    def from_message(cls, message):
        return cls(
            message.unique_id,
            message.name,
            VectorState.from_message(message.position),
            VectorState.from_message(message.size),
            message.heading,
        )

    @classmethod
    def interpolate(cls, first, second, ratio):
        reference = first if ratio <= 0.5 else second
        return cls(
            reference.unique_id,
            reference.name,
            VectorState.interpolate(first.position, second.position, ratio),
            VectorState.interpolate(first.size, second.size, ratio),
            interpolate_angle_degrees(first.heading, second.heading, ratio),
        )


class ObjectFrame:
    __slots__ = ("stamp", "objects")

    def __init__(self, stamp, objects):
        self.stamp = float(stamp)
        self.objects = objects

    @classmethod
    def from_message(cls, message):
        return cls(
            message.header.stamp.to_sec(),
            {
                int(item.unique_id): ObjectState.from_message(item)
                for item in message.npc_list
            },
        )

    @classmethod
    def interpolate(cls, first, second, stamp):
        ratio = interpolation_ratio(first.stamp, second.stamp, stamp)
        objects = {}
        for key in set(first.objects) | set(second.objects):
            before = first.objects.get(key)
            after = second.objects.get(key)
            if before is not None and after is not None:
                objects[key] = ObjectState.interpolate(before, after, ratio)
            elif before is not None and ratio <= 0.5:
                objects[key] = before
            elif after is not None and ratio > 0.5:
                objects[key] = after
        return cls(stamp, objects)


class CameraVehicleDatasetRecorder:
    def __init__(self):
        self.topics = rospy.get_param("~topics")
        self.camera = rospy.get_param("~camera")
        sync = rospy.get_param("~sync", {})
        recording = rospy.get_param("~recording", {})
        filtering = rospy.get_param("~filtering", {})

        self.buffer_seconds = float(sync.get("buffer_seconds", 2.0))
        self.max_wait_seconds = float(sync.get("max_wait_seconds", 0.15))
        self.max_nearest_seconds = float(sync.get("max_nearest_seconds", 0.06))
        self.record_every_n = max(1, int(recording.get("record_every_n", 6)))
        self.empty_frame_every_n = max(
            1, int(recording.get("empty_frame_every_n", 10))
        )
        self.jpeg_quality = min(100, max(1, int(recording.get("jpeg_quality", 95))))
        self.save_overlays = bool(recording.get("save_overlays", True))
        self.enabled = bool(recording.get("enabled", True))
        self.class_id = int(filtering.get("class_id", 0))
        self.class_name = str(filtering.get("class_name", "Car"))
        self.position_is_bottom = bool(
            filtering.get("position_is_bottom", True)
        )
        self.minimum_range = float(filtering.get("minimum_range_m", 1.0))
        self.maximum_range = float(filtering.get("maximum_range_m", 80.0))
        self.minimum_box_width = float(
            filtering.get("minimum_box_width_px", 12.0)
        )
        self.minimum_box_height = float(
            filtering.get("minimum_box_height_px", 12.0)
        )
        self.minimum_inside_ratio = float(
            filtering.get("minimum_inside_ratio", 0.30)
        )
        self._validate_configuration()

        self.output_dir = Path(rospy.get_param("~output_dir")).expanduser().resolve()
        self.session_id = str(rospy.get_param("~session_id", "camera_session"))
        self.directories = {
            "images": self.output_dir / "images",
            "labels": self.output_dir / "labels",
            "metadata": self.output_dir / "metadata",
            "overlays": self.output_dir / "overlays",
        }
        for name, directory in self.directories.items():
            if name != "overlays" or self.save_overlays:
                directory.mkdir(parents=True, exist_ok=True)
        self.manifest_path = self.output_dir / "manifest.jsonl"

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.dispatch_lock = threading.Lock()
        self.ego_buffer = deque()
        self.object_buffer = deque()
        self.pending_images = deque()
        self.received_images = 0
        self.candidate_images = 0
        self.empty_candidates = 0
        self.saved_frames = self._next_frame_number()
        self.saved_this_run = 0
        self.dropped_unsynchronised = 0

        self.overlay_pub = rospy.Publisher(
            self.topics["overlay"], Image, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            self.topics["status"], String, queue_size=1, latch=True
        )
        self.image_sub = rospy.Subscriber(
            self.topics["image"], Image, self.image_callback,
            queue_size=10, buff_size=24 * 1024 * 1024,
        )
        self.ego_sub = rospy.Subscriber(
            self.topics["ego"], EgoVehicleStatus, self.ego_callback, queue_size=100
        )
        self.object_sub = rospy.Subscriber(
            self.topics["objects"], ObjectStatusList,
            self.object_callback, queue_size=100,
        )
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_callback)
        self.enable_service = rospy.Service(
            "~set_enabled", SetBool, self.set_enabled_callback
        )
        rospy.on_shutdown(self.shutdown)
        self.publish_status()
        rospy.loginfo(
            "Camera vehicle recorder ready: image=%s output=%s stride=%d enabled=%s",
            self.topics["image"], self.output_dir, self.record_every_n, self.enabled,
        )

    def _validate_configuration(self):
        for key in ("position", "rotation", "width", "height", "fov"):
            if key not in self.camera:
                raise ValueError("missing camera/{}".format(key))
        if len(self.camera["position"]) != 3 or len(self.camera["rotation"]) != 3:
            raise ValueError("camera position and rotation must each have 3 values")
        if int(self.camera["width"]) <= 0 or int(self.camera["height"]) <= 0:
            raise ValueError("camera dimensions must be positive")
        if not 0.0 <= self.minimum_inside_ratio <= 1.0:
            raise ValueError("minimum_inside_ratio must be in [0, 1]")
        if self.minimum_range < 0.0 or self.maximum_range <= self.minimum_range:
            raise ValueError("invalid object range")

    def _next_frame_number(self):
        maximum = -1
        if self.manifest_path.is_file():
            with self.manifest_path.open(encoding="utf-8") as stream:
                for line in stream:
                    try:
                        maximum = max(maximum, int(json.loads(line)["frame_id"]))
                    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                        continue
        return maximum + 1

    @staticmethod
    def append_in_order(buffer, sample):
        if not buffer or sample.stamp >= buffer[-1].stamp:
            buffer.append(sample)
            return
        for index in range(len(buffer) - 1, -1, -1):
            if buffer[index].stamp <= sample.stamp:
                buffer.insert(index + 1, sample)
                return
        buffer.appendleft(sample)

    def prune(self, buffer):
        if not buffer:
            return
        cutoff = buffer[-1].stamp - self.buffer_seconds
        while len(buffer) > 2 and buffer[1].stamp < cutoff:
            buffer.popleft()

    @staticmethod
    def find_bracket(buffer, stamp):
        previous = None
        for sample in buffer:
            if sample.stamp == stamp:
                return sample, sample
            if sample.stamp > stamp:
                return None if previous is None else (previous, sample)
            previous = sample
        return None

    def select_samples(self, buffer, stamp, timed_out):
        bracket = self.find_bracket(buffer, stamp)
        if bracket is not None:
            return bracket, "interpolated"
        if not timed_out or not buffer:
            return None, None
        nearest = min(buffer, key=lambda sample: abs(sample.stamp - stamp))
        if abs(nearest.stamp - stamp) > self.max_nearest_seconds:
            return None, None
        return (nearest, nearest), "nearest_fallback"

    def image_callback(self, message):
        self.received_images += 1
        if (self.received_images - 1) % self.record_every_n:
            return
        with self.lock:
            self.pending_images.append((message, time.monotonic()))
        self.dispatch()

    def ego_callback(self, message):
        sample = EgoState.from_message(message)
        with self.lock:
            self.append_in_order(self.ego_buffer, sample)
            self.prune(self.ego_buffer)
        self.dispatch()

    def object_callback(self, message):
        sample = ObjectFrame.from_message(message)
        with self.lock:
            self.append_in_order(self.object_buffer, sample)
            self.prune(self.object_buffer)
        self.dispatch()

    def timer_callback(self, _event):
        self.dispatch()

    def dispatch(self):
        if not self.dispatch_lock.acquire(False):
            return
        try:
            self._dispatch()
        finally:
            self.dispatch_lock.release()

    def _dispatch(self):
        work = []
        now = time.monotonic()
        with self.lock:
            while self.pending_images:
                image, arrival = self.pending_images[0]
                stamp = image.header.stamp.to_sec()
                if stamp <= 0.0:
                    rospy.logwarn_throttle(2.0, "Dropping camera frame with zero stamp")
                    self.pending_images.popleft()
                    self.dropped_unsynchronised += 1
                    continue
                timed_out = now - arrival >= self.max_wait_seconds
                ego_samples, ego_mode = self.select_samples(
                    self.ego_buffer, stamp, timed_out
                )
                object_samples, object_mode = self.select_samples(
                    self.object_buffer, stamp, timed_out
                )
                if ego_samples is None or object_samples is None:
                    if timed_out:
                        self.pending_images.popleft()
                        self.dropped_unsynchronised += 1
                        rospy.logwarn_throttle(
                            2.0, "Dropping camera frame without matching Ego/Object truth"
                        )
                        continue
                    break
                self.pending_images.popleft()
                work.append(
                    (image, ego_samples, object_samples, ego_mode, object_mode)
                )
        for item in work:
            stamp = item[0].header.stamp.to_sec()
            ego = EgoState.interpolate(item[1][0], item[1][1], stamp)
            objects = ObjectFrame.interpolate(item[2][0], item[2][1], stamp)
            self.process(item[0], ego, objects, item[3], item[4])

    def labels_for_objects(self, ego, objects):
        labels = []
        rejected = {"range": 0, "projection": 0, "small": 0}
        for obj in objects.objects.values():
            center, size, yaw = world_object_to_vehicle(
                obj.position.as_list(), obj.heading, obj.size.as_list(),
                ego.position.as_list(), ego.heading, self.position_is_bottom,
            )
            distance = float(math.hypot(center[0], center[1]))
            if distance < self.minimum_range or distance > self.maximum_range:
                rejected["range"] += 1
                continue
            corners = make_vehicle_box_corners(center, size, yaw)
            result = projected_box(
                corners, self.camera, self.minimum_inside_ratio
            )
            if result is None:
                rejected["projection"] += 1
                continue
            box, inside_ratio = result
            if (
                box[2] - box[0] < self.minimum_box_width
                or box[3] - box[1] < self.minimum_box_height
            ):
                rejected["small"] += 1
                continue
            labels.append(
                {
                    "class_id": self.class_id,
                    "class_name": self.class_name,
                    "object_id": obj.unique_id,
                    "object_name": obj.name,
                    "bbox_xyxy": [float(value) for value in box],
                    "bbox_yolo": list(
                        xyxy_to_yolo(
                            box, self.camera["width"], self.camera["height"]
                        )
                    ),
                    "vehicle_center": center.tolist(),
                    "vehicle_size": size.tolist(),
                    "vehicle_yaw_rad": float(yaw),
                    "distance_m": distance,
                    "inside_ratio": float(inside_ratio),
                }
            )
        return labels, rejected

    def process(self, message, ego, objects, ego_mode, object_mode):
        self.candidate_images += 1
        if (
            int(message.width) != int(self.camera["width"])
            or int(message.height) != int(self.camera["height"])
        ):
            rospy.logerr_throttle(
                2.0,
                "Camera image is %dx%d but calibration is %dx%d; frame rejected",
                message.width,
                message.height,
                self.camera["width"],
                self.camera["height"],
            )
            return
        labels, rejected = self.labels_for_objects(ego, objects)
        if not labels:
            self.empty_candidates += 1
            if (self.empty_candidates - 1) % self.empty_frame_every_n:
                return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logerr_throttle(2.0, "Could not convert camera image: %s", error)
            return
        overlay = self.draw_overlay(image, labels)
        overlay_message = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        overlay_message.header = message.header
        self.overlay_pub.publish(overlay_message)
        if not self.enabled:
            return
        self.save_sample(
            image, overlay, message, ego, labels, rejected, ego_mode, object_mode
        )

    def draw_overlay(self, image, labels):
        overlay = image.copy()
        for label in labels:
            x1, y1, x2, y2 = [int(round(value)) for value in label["bbox_xyxy"]]
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                overlay,
                "Car id={} {:.1f}m".format(
                    label["object_id"], label["distance_m"]
                ),
                (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 255, 0), 2, cv2.LINE_AA,
            )
        return overlay

    @staticmethod
    def atomic_text(path, content):
        temporary = Path(str(path) + ".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(path))

    def atomic_jpeg(self, path, image):
        ok, encoded = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            raise RuntimeError("OpenCV JPEG encoding failed")
        temporary = Path(str(path) + ".tmp")
        with temporary.open("wb") as stream:
            stream.write(encoded.tobytes())
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(path))

    def save_sample(
        self, image, overlay, message, ego, labels, rejected, ego_mode, object_mode
    ):
        frame_id = "{:06d}".format(self.saved_frames)
        image_path = self.directories["images"] / (frame_id + ".jpg")
        label_path = self.directories["labels"] / (frame_id + ".txt")
        metadata_path = self.directories["metadata"] / (frame_id + ".json")
        overlay_path = self.directories["overlays"] / (frame_id + ".jpg")
        stamp = message.header.stamp
        yolo_lines = [
            "{} {:.9f} {:.9f} {:.9f} {:.9f}".format(
                label["class_id"], *label["bbox_yolo"]
            )
            for label in labels
        ]
        metadata = {
            "frame_id": frame_id,
            "session_id": self.session_id,
            "stamp": {"secs": stamp.secs, "nsecs": stamp.nsecs},
            "image": {
                "width": int(message.width), "height": int(message.height),
                "topic": self.topics["image"],
            },
            "sync_mode": (
                "interpolated"
                if ego_mode == "interpolated" and object_mode == "interpolated"
                else "nearest_fallback"
            ),
            "ego": {
                "position": ego.position.as_list(), "heading_deg": ego.heading
            },
            "num_labels": len(labels),
            "labels": labels,
            "rejected": rejected,
        }
        try:
            self.atomic_jpeg(image_path, image)
            self.atomic_text(label_path, "\n".join(yolo_lines) + ("\n" if yolo_lines else ""))
            self.atomic_text(
                metadata_path,
                json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            )
            if self.save_overlays:
                self.atomic_jpeg(overlay_path, overlay)
            manifest = {
                "frame_id": frame_id,
                "session_id": self.session_id,
                "stamp_ns": int(stamp.to_nsec()),
                "num_labels": len(labels),
                "image": str(image_path.relative_to(self.output_dir)),
                "label": str(label_path.relative_to(self.output_dir)),
            }
            with self.manifest_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(manifest, ensure_ascii=False) + "\n")
                stream.flush()
                os.fsync(stream.fileno())
        except (OSError, RuntimeError) as error:
            rospy.logerr("Could not save camera dataset frame %s: %s", frame_id, error)
            return
        self.saved_frames += 1
        self.saved_this_run += 1
        self.publish_status()
        rospy.loginfo_throttle(
            2.0, "Camera dataset saved=%d frame=%s cars=%d",
            self.saved_this_run, frame_id, len(labels),
        )

    def set_enabled_callback(self, request):
        self.enabled = bool(request.data)
        self.publish_status()
        return SetBoolResponse(
            success=True,
            message="camera dataset recording {}".format(
                "enabled" if self.enabled else "disabled"
            ),
        )

    def publish_status(self):
        status = {
            "enabled": self.enabled,
            "output_dir": str(self.output_dir),
            "session_id": self.session_id,
            "received_images": self.received_images,
            "candidate_images": self.candidate_images,
            "saved_this_run": self.saved_this_run,
            "next_frame_id": "{:06d}".format(self.saved_frames),
            "dropped_unsynchronised": self.dropped_unsynchronised,
        }
        self.status_pub.publish(
            String(data=json.dumps(status, ensure_ascii=False, separators=(",", ":")))
        )

    def shutdown(self):
        self.publish_status()


def main():
    rospy.init_node("camera_vehicle_dataset_recorder")
    CameraVehicleDatasetRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
