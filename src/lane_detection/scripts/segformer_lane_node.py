#!/usr/bin/env python3
"""Run the trained SegFormer model on an existing ROS front-camera topic."""

import json
import os
import sys
import threading
import time
from pathlib import Path

# The model runs in the lane_tool Conda environment.  ROS Noetic's Python
# packages remain available from the system locations below.
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
import segmentation_models_pytorch as smp
import torch
import torch.nn.functional as F
from sensor_msgs.msg import Image
from std_msgs.msg import String


# OpenCV BGR palette.  These colors match the lane_tool label visualizer.
PALETTE = np.array(
    [
        [0, 0, 0],        # 0: background
        [0, 255, 0],      # 1: class_1
        [0, 255, 255],    # 2: class_2
        [255, 0, 255],    # 3: class_3
    ],
    dtype=np.uint8,
)
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class SegFormerLaneNode:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/around_view/camera/front/image_raw"
        )
        self.mask_topic = rospy.get_param("~mask_topic", "/lane_detection/mask")
        self.overlay_topic = rospy.get_param(
            "~overlay_topic", "/lane_detection/overlay"
        )
        self.diagnostics_topic = rospy.get_param(
            "~diagnostics_topic", "/lane_detection/diagnostics"
        )
        self.model_path = Path(rospy.get_param("~model_path")).expanduser()
        self.encoder = str(rospy.get_param("~encoder", "mit_b1"))
        self.num_classes = int(rospy.get_param("~num_classes", 4))
        self.input_width = int(rospy.get_param("~input_width", 1280))
        self.input_height = int(rospy.get_param("~input_height", 736))
        self.content_width = int(rospy.get_param("~content_width", 1280))
        self.content_height = int(rospy.get_param("~content_height", 720))
        self.overlay_alpha = float(rospy.get_param("~overlay_alpha", 0.62))
        self.max_inference_hz = float(rospy.get_param("~max_inference_hz", 0.0))

        requested_device = str(rospy.get_param("~device", "cuda"))
        if requested_device.startswith("cuda") and not torch.cuda.is_available():
            rospy.logwarn("CUDA unavailable; lane detection is falling back to CPU")
            requested_device = "cpu"
        self.device = torch.device(requested_device)
        self.use_amp = self.device.type == "cuda"

        self._validate_configuration()
        self.model = self._load_model()

        self.mask_publisher = rospy.Publisher(self.mask_topic, Image, queue_size=1)
        self.overlay_publisher = rospy.Publisher(
            self.overlay_topic, Image, queue_size=1
        )
        self.diagnostics_publisher = rospy.Publisher(
            self.diagnostics_topic, String, queue_size=1
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
            "Lane SegFormer ready: input=%s model=%s device=%s output=%s",
            self.input_topic,
            self.model_path,
            self.device,
            self.overlay_topic,
        )

    def _validate_configuration(self):
        if not self.model_path.is_file():
            raise FileNotFoundError("Lane model not found: {}".format(self.model_path))
        if self.input_width % 32 or self.input_height % 32:
            raise ValueError("SegFormer input width and height must be divisible by 32")
        if not (0 < self.content_width <= self.input_width):
            raise ValueError("content_width must be in (0, input_width]")
        if not (0 < self.content_height <= self.input_height):
            raise ValueError("content_height must be in (0, input_height]")
        if not (2 <= self.num_classes <= len(PALETTE)):
            raise ValueError("num_classes must be between 2 and {}".format(len(PALETTE)))
        if not (0.0 <= self.overlay_alpha <= 1.0):
            raise ValueError("overlay_alpha must be between 0 and 1")

    def _load_model(self):
        model = smp.Segformer(
            encoder_name=self.encoder,
            encoder_weights=None,
            in_channels=3,
            classes=self.num_classes,
            activation=None,
        )
        payload = torch.load(self.model_path, map_location="cpu", weights_only=False)
        if isinstance(payload, dict) and "model_state_dict" in payload:
            payload = payload["model_state_dict"]
        model.load_state_dict(payload, strict=True)
        model.to(self.device)
        model.eval()
        if self.device.type == "cuda":
            torch.backends.cudnn.benchmark = True
        return model

    def _image_callback(self, message):
        with self.lock:
            self.received_frames += 1
            if self.latest_message is not None:
                self.dropped_frames += 1
            self.latest_message = message
            self.frame_event.set()

    @staticmethod
    def _image_message_to_bgr(message):
        """Convert common 8-bit ROS Image encodings without cv_bridge.

        The lane model runs under a Conda Python while ROS Noetic's cv_bridge
        extension was compiled against the system Python.  Direct conversion
        avoids that ABI boundary and also respects padded ROS row strides.
        """
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
    def _array_to_image_message(array, encoding, header):
        if array.dtype != np.uint8:
            raise ValueError("Published lane image must be uint8")
        contiguous = np.ascontiguousarray(array)
        message = Image()
        message.header = header
        message.height = contiguous.shape[0]
        message.width = contiguous.shape[1]
        message.encoding = encoding
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

    def _preprocess(self, frame):
        resized = cv2.resize(
            frame,
            (self.content_width, self.content_height),
            interpolation=cv2.INTER_LINEAR,
        )
        pad_x = self.input_width - self.content_width
        pad_y = self.input_height - self.content_height
        left = pad_x // 2
        top = pad_y // 2
        canvas = cv2.copyMakeBorder(
            resized,
            top,
            pad_y - top,
            left,
            pad_x - left,
            cv2.BORDER_CONSTANT,
            value=(0, 0, 0),
        )
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        rgb = (rgb - MEAN) / STD
        tensor = torch.from_numpy(np.ascontiguousarray(rgb.transpose(2, 0, 1)))
        tensor = tensor.unsqueeze(0).to(self.device, non_blocking=True)
        return tensor, left, top

    @torch.inference_mode()
    def _infer(self, frame):
        tensor, left, top = self._preprocess(frame)
        with torch.amp.autocast(self.device.type, enabled=self.use_amp):
            logits = self.model(tensor)
            if logits.shape[-2:] != (self.input_height, self.input_width):
                logits = F.interpolate(
                    logits,
                    size=(self.input_height, self.input_width),
                    mode="bilinear",
                    align_corners=False,
                )
        mask = torch.argmax(logits, dim=1)[0].byte().cpu().numpy()
        mask = mask[
            top : top + self.content_height,
            left : left + self.content_width,
        ]
        if mask.shape != frame.shape[:2]:
            mask = cv2.resize(
                mask,
                (frame.shape[1], frame.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )
        return mask

    def _make_overlay(self, frame, mask):
        color_mask = PALETTE[mask]
        blended = cv2.addWeighted(
            frame, 1.0 - self.overlay_alpha, color_mask, self.overlay_alpha, 0.0
        )
        overlay = frame.copy()
        foreground = mask != 0
        overlay[foreground] = blended[foreground]

        legend = (
            ("class_1", tuple(int(v) for v in PALETTE[1])),
            ("class_2", tuple(int(v) for v in PALETTE[2])),
            ("class_3", tuple(int(v) for v in PALETTE[3])),
        )
        for index, (label, color) in enumerate(legend):
            y = 28 + index * 28
            cv2.rectangle(overlay, (12, y - 15), (30, y + 3), color, -1)
            cv2.putText(
                overlay,
                label,
                (38, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        return overlay

    def _publish(self, source_message, mask, overlay, latency_ms):
        mask_message = self._array_to_image_message(
            mask, "mono8", source_message.header
        )
        overlay_message = self._array_to_image_message(
            overlay, "bgr8", source_message.header
        )
        self.mask_publisher.publish(mask_message)
        self.overlay_publisher.publish(overlay_message)

        class_pixels = np.bincount(
            mask.reshape(-1), minlength=self.num_classes
        ).tolist()
        diagnostics = {
            "received_frames": self.received_frames,
            "processed_frames": self.processed_frames,
            "dropped_frames": self.dropped_frames,
            "latency_ms": round(latency_ms, 2),
            "fps": round(self.fps_ema, 2),
            "class_pixels": class_pixels,
            "source_width": int(mask.shape[1]),
            "source_height": int(mask.shape[0]),
            "device": str(self.device),
        }
        self.diagnostics_publisher.publish(
            String(data=json.dumps(diagnostics, ensure_ascii=False))
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
                if remaining > 0:
                    time.sleep(remaining)

            started = time.perf_counter()
            self.last_start_time = started
            try:
                frame = self._image_message_to_bgr(message)
                mask = self._infer(frame)
                overlay = self._make_overlay(frame, mask)
                if self.device.type == "cuda":
                    torch.cuda.synchronize(self.device)
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
                self._publish(message, mask, overlay, latency_ms)
                rospy.loginfo_throttle(
                    2.0,
                    "Lane frames=%d dropped=%d latency=%.1fms fps=%.1f",
                    self.processed_frames,
                    self.dropped_frames,
                    latency_ms,
                    self.fps_ema,
                )
            except Exception as error:
                rospy.logerr_throttle(2.0, "Lane inference failed: %s", error)


def main():
    rospy.init_node("lane_detection")
    SegFormerLaneNode()
    rospy.spin()


if __name__ == "__main__":
    main()
