#!/usr/bin/env python3
"""Run the trained camera vehicle detector on a ROS Image topic."""

import json
import os
import sys
import threading
import time
from pathlib import Path

for ros_python_path in (
    "/opt/ros/noetic/lib/python3/dist-packages",
    "/usr/lib/python3/dist-packages",
):
    if ros_python_path not in sys.path:
        sys.path.append(ros_python_path)

os.environ.setdefault("NO_ALBUMENTATIONS_UPDATE", "1")

import cv2
import numpy as np
import rospy
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class CameraVehicleDetector:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/around_view/camera/front/image_raw"
        )
        self.overlay_topic = rospy.get_param(
            "~overlay_topic", "/camera_vehicle_detection/overlay"
        )
        self.detections_topic = rospy.get_param(
            "~detections_topic", "/camera_vehicle_detection/detections"
        )
        self.model_path = Path(rospy.get_param("~model_path")).expanduser()
        self.image_size = int(rospy.get_param("~image_size", 1280))
        self.confidence = float(rospy.get_param("~confidence", 0.25))
        self.iou_threshold = float(rospy.get_param("~iou_threshold", 0.70))
        self.max_detections = int(rospy.get_param("~max_detections", 100))
        self.max_inference_hz = float(rospy.get_param("~max_inference_hz", 0.0))
        self.line_width = int(rospy.get_param("~line_width", 2))

        requested_device = str(rospy.get_param("~device", "0"))
        if requested_device != "cpu" and not torch.cuda.is_available():
            rospy.logwarn("CUDA unavailable; vehicle detection is falling back to CPU")
            requested_device = "cpu"
        self.device = requested_device
        self.use_half = self.device != "cpu"

        self._validate_configuration()
        self.model = YOLO(str(self.model_path))
        if self.device != "cpu":
            torch.backends.cudnn.benchmark = True
        self._warm_up()

        self.overlay_publisher = rospy.Publisher(
            self.overlay_topic, Image, queue_size=1
        )
        self.detections_publisher = rospy.Publisher(
            self.detections_topic, String, queue_size=1
        )

        self.lock = threading.Lock()
        self.frame_event = threading.Event()
        self.latest_message = None
        self.received_frames = 0
        self.processed_frames = 0
        self.dropped_frames = 0
        self.fps_ema = 0.0
        self.last_finish_time = None
        self.last_start_time = 0.0

        self.subscriber = rospy.Subscriber(
            self.input_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=24 * 1024 * 1024,
            tcp_nodelay=True,
        )
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        rospy.loginfo(
            "Camera vehicle detector ready: input=%s model=%s device=%s overlay=%s",
            self.input_topic,
            self.model_path,
            self.device,
            self.overlay_topic,
        )

    def _validate_configuration(self):
        if not self.model_path.is_file():
            raise FileNotFoundError("Vehicle model not found: {}".format(self.model_path))
        if self.image_size <= 0 or self.image_size % 32:
            raise ValueError("image_size must be a positive multiple of 32")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be between 0 and 1")
        if not 0.0 <= self.iou_threshold <= 1.0:
            raise ValueError("iou_threshold must be between 0 and 1")

    def _warm_up(self):
        sample = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.model.predict(
            source=sample,
            imgsz=self.image_size,
            conf=self.confidence,
            iou=self.iou_threshold,
            device=self.device,
            half=self.use_half,
            max_det=self.max_detections,
            verbose=False,
        )
        if self.device != "cpu":
            torch.cuda.synchronize()

    def _image_callback(self, message):
        with self.lock:
            self.received_frames += 1
            if self.latest_message is not None:
                self.dropped_frames += 1
            self.latest_message = message
            self.frame_event.set()

    @staticmethod
    def _image_message_to_bgr(message):
        encoding = message.encoding.lower()
        channels_by_encoding = {
            "mono8": 1,
            "8uc1": 1,
            "bgr8": 3,
            "rgb8": 3,
            "bgra8": 4,
            "rgba8": 4,
        }
        if encoding not in channels_by_encoding:
            raise ValueError("Unsupported camera encoding: {}".format(message.encoding))
        channels = channels_by_encoding[encoding]
        expected_row_bytes = int(message.width) * channels
        if int(message.step) < expected_row_bytes:
            raise ValueError("ROS Image step is smaller than its pixel width")
        raw = np.frombuffer(message.data, dtype=np.uint8)
        required_bytes = int(message.height) * int(message.step)
        if raw.size < required_bytes:
            raise ValueError("ROS Image data is shorter than height * step")
        rows = raw[:required_bytes].reshape(int(message.height), int(message.step))
        pixels = rows[:, :expected_row_bytes]
        if channels == 1:
            mono = pixels.reshape(int(message.height), int(message.width))
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        image = pixels.reshape(int(message.height), int(message.width), channels)
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return np.ascontiguousarray(image)

    @staticmethod
    def _array_to_image_message(array, header):
        contiguous = np.ascontiguousarray(array, dtype=np.uint8)
        message = Image()
        message.header = header
        message.height = contiguous.shape[0]
        message.width = contiguous.shape[1]
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = contiguous.strides[0]
        message.data = contiguous.tobytes()
        return message

    def _take_latest_message(self):
        with self.lock:
            message = self.latest_message
            self.latest_message = None
            self.frame_event.clear()
            return message

    def _infer(self, frame):
        result = self.model.predict(
            source=frame,
            imgsz=self.image_size,
            conf=self.confidence,
            iou=self.iou_threshold,
            device=self.device,
            half=self.use_half,
            max_det=self.max_detections,
            verbose=False,
        )[0]
        if self.device != "cpu":
            torch.cuda.synchronize()

        detections = []
        boxes = result.boxes
        if boxes is not None and len(boxes):
            xyxy = boxes.xyxy.detach().cpu().numpy()
            confidences = boxes.conf.detach().cpu().numpy()
            class_ids = boxes.cls.detach().cpu().numpy().astype(int)
            for coordinates, confidence, class_id in zip(
                xyxy, confidences, class_ids
            ):
                x1, y1, x2, y2 = (float(value) for value in coordinates)
                detections.append(
                    {
                        "class_id": int(class_id),
                        "class_name": str(result.names.get(int(class_id), class_id)),
                        "confidence": round(float(confidence), 5),
                        "bbox_xyxy": [round(x1, 2), round(y1, 2), round(x2, 2), round(y2, 2)],
                        "center_xy": [round((x1 + x2) / 2.0, 2), round((y1 + y2) / 2.0, 2)],
                        "size_wh": [round(x2 - x1, 2), round(y2 - y1, 2)],
                    }
                )
        overlay = result.plot(
            conf=True,
            labels=True,
            boxes=True,
            line_width=self.line_width,
        )
        return overlay, detections

    def _publish(self, source_message, overlay, detections, latency_ms):
        self.overlay_publisher.publish(
            self._array_to_image_message(overlay, source_message.header)
        )
        payload = {
            "stamp_ns": int(source_message.header.stamp.to_nsec()),
            "frame_id": str(source_message.header.frame_id),
            "source_width": int(source_message.width),
            "source_height": int(source_message.height),
            "num_detections": len(detections),
            "detections": detections,
            "latency_ms": round(latency_ms, 2),
            "fps": round(self.fps_ema, 2),
            "received_frames": self.received_frames,
            "processed_frames": self.processed_frames,
            "dropped_frames": self.dropped_frames,
            "device": self.device,
        }
        self.detections_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _worker_loop(self):
        while not rospy.is_shutdown():
            if not self.frame_event.wait(timeout=0.25):
                continue
            message = self._take_latest_message()
            if message is None:
                continue
            if self.max_inference_hz > 0.0:
                minimum_period = 1.0 / self.max_inference_hz
                remaining = minimum_period - (time.perf_counter() - self.last_start_time)
                if remaining > 0.0:
                    time.sleep(remaining)

            started = time.perf_counter()
            self.last_start_time = started
            try:
                frame = self._image_message_to_bgr(message)
                overlay, detections = self._infer(frame)
                finished = time.perf_counter()
                latency_ms = (finished - started) * 1000.0
                self.processed_frames += 1
                if self.last_finish_time is not None and finished > self.last_finish_time:
                    instant_fps = 1.0 / (finished - self.last_finish_time)
                    self.fps_ema = (
                        instant_fps
                        if self.fps_ema == 0.0
                        else 0.9 * self.fps_ema + 0.1 * instant_fps
                    )
                self.last_finish_time = finished
                self._publish(message, overlay, detections, latency_ms)
                rospy.loginfo_throttle(
                    2.0,
                    "Vehicle detection frames=%d dropped=%d objects=%d latency=%.1fms fps=%.1f",
                    self.processed_frames,
                    self.dropped_frames,
                    len(detections),
                    latency_ms,
                    self.fps_ema,
                )
            except Exception as error:
                rospy.logerr_throttle(2.0, "Vehicle inference failed: %s", error)


def main():
    rospy.init_node("camera_vehicle_detector")
    CameraVehicleDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
