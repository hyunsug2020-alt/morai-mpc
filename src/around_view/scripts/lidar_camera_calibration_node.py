#!/usr/bin/env python3
"""Overlay MORAI VLP-16 points on the three fixed around-view cameras."""

import os
import threading

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import yaml
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2

from around_view import project_lidar_points


class LidarCameraCalibrationNode:
    CAMERA_NAMES = ("front", "left", "right")

    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = {name: None for name in self.CAMERA_NAMES}
        self.frame_stamps = {
            name: rospy.Time(0) for name in self.CAMERA_NAMES
        }
        self.points = None
        self.point_stamp = rospy.Time(0)

        self.point_cloud_topic = rospy.get_param(
            "~point_cloud_topic", "/velodyne_points"
        )
        self.lidar = rospy.get_param("~lidar")
        self.cameras = rospy.get_param("~cameras")
        self.correction = rospy.get_param(
            "~lidar_correction",
            {
                "translation": [0.0, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0],
            },
        )
        projection = rospy.get_param("~projection", {})
        self.minimum_depth = float(projection.get("minimum_depth", 0.5))
        self.maximum_depth = float(projection.get("maximum_depth", 80.0))
        self.point_stride = max(1, int(projection.get("point_stride", 2)))
        self.maximum_points = max(
            100, int(projection.get("maximum_points", 30000))
        )
        self.point_radius = max(1, int(projection.get("point_radius", 2)))
        self.publish_rate = max(
            1.0, float(projection.get("publish_rate", 10.0))
        )
        self.maximum_time_difference = max(
            0.0, float(projection.get("maximum_time_difference", 0.30))
        )
        self.show_window = bool(rospy.get_param("~show_window", True))
        self.calibration_file = os.path.expanduser(
            rospy.get_param(
                "~calibration_file",
                "~/.ros/around_view/lidar_camera_calibration.yaml",
            )
        )
        self.gui_available = self.show_window

        self._validate_configuration()
        self._load_saved_correction()

        self.publishers = {}
        self.image_subscribers = []
        for name in self.CAMERA_NAMES:
            config = self.cameras[name]
            self.publishers[name] = rospy.Publisher(
                config["output_topic"], Image, queue_size=1
            )
            self.image_subscribers.append(
                rospy.Subscriber(
                    config["topic"],
                    Image,
                    self._image_callback,
                    callback_args=name,
                    queue_size=1,
                    buff_size=2**24,
                )
            )

        self.point_subscriber = rospy.Subscriber(
            self.point_cloud_topic,
            PointCloud2,
            self._point_cloud_callback,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.on_shutdown(self.shutdown)

        self._setup_windows()
        rospy.loginfo(
            "LiDAR-camera calibration ready: cloud=%s, cameras=%s",
            self.point_cloud_topic,
            ", ".join(self.CAMERA_NAMES),
        )

    def _validate_configuration(self):
        for key in ("position", "rotation"):
            if key not in self.lidar or len(self.lidar[key]) != 3:
                raise ValueError("lidar/{} must contain three values".format(key))
        for name in self.CAMERA_NAMES:
            if name not in self.cameras:
                raise ValueError("Missing camera config: {}".format(name))
            config = self.cameras[name]
            for key in (
                "topic",
                "output_topic",
                "position",
                "rotation",
                "width",
                "height",
                "fov",
            ):
                if key not in config:
                    raise ValueError(
                        "Missing cameras/{}/{}".format(name, key)
                    )

    def _load_saved_correction(self):
        if not os.path.isfile(self.calibration_file):
            return
        try:
            with open(self.calibration_file, "r") as stream:
                saved = yaml.safe_load(stream) or {}
            correction = saved.get("lidar_correction", {})
            for key in ("translation", "rotation"):
                values = correction.get(key)
                if isinstance(values, list) and len(values) == 3:
                    self.correction[key] = [float(value) for value in values]
            rospy.loginfo(
                "Loaded LiDAR-camera correction: %s", self.calibration_file
            )
        except (OSError, ValueError, yaml.YAMLError) as error:
            rospy.logwarn("Could not load LiDAR-camera correction: %s", error)

    def _image_callback(self, message, camera_name):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            )
        except CvBridgeError as error:
            rospy.logwarn_throttle(3.0, "Image conversion failed: %s", error)
            return
        stamp = message.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
        with self.lock:
            self.frames[camera_name] = frame
            self.frame_stamps[camera_name] = stamp

    def _point_cloud_callback(self, message):
        points = []
        for index, point in enumerate(
            point_cloud2.read_points(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        ):
            if index % self.point_stride:
                continue
            points.append(point)
            if len(points) >= self.maximum_points:
                break

        if not points:
            rospy.logwarn_throttle(3.0, "No finite points in %s", self.point_cloud_topic)
            return

        stamp = message.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
        with self.lock:
            self.points = np.asarray(points, dtype=np.float32)
            self.point_stamp = stamp

    def _setup_windows(self):
        if not self.gui_available:
            return
        try:
            for name in self.CAMERA_NAMES:
                cv2.namedWindow(
                    "{} LiDAR overlay".format(name), cv2.WINDOW_NORMAL
                )
        except cv2.error as error:
            rospy.logwarn("OpenCV windows disabled: %s", error)
            self.gui_available = False

    def _draw_points(self, frame, pixels, depths):
        if not len(pixels):
            return
        denominator = max(
            1e-6, self.maximum_depth - self.minimum_depth
        )
        normalized = np.clip(
            (depths - self.minimum_depth) / denominator, 0.0, 1.0
        )
        color_values = np.asarray(
            np.round((1.0 - normalized) * 255.0), dtype=np.uint8
        )
        colors = cv2.applyColorMap(
            color_values.reshape((-1, 1)), cv2.COLORMAP_TURBO
        ).reshape((-1, 3))

        # Draw distant points first so nearby geometry remains visible.
        for index in np.argsort(depths)[::-1]:
            pixel = pixels[index]
            color = tuple(int(value) for value in colors[index])
            cv2.circle(
                frame,
                (int(round(pixel[0])), int(round(pixel[1]))),
                self.point_radius,
                color,
                -1,
                lineType=cv2.LINE_AA,
            )

    def _render(self):
        with self.lock:
            points = None if self.points is None else self.points.copy()
            point_stamp = self.point_stamp
            frames = {
                name: None if frame is None else frame.copy()
                for name, frame in self.frames.items()
            }
            frame_stamps = dict(self.frame_stamps)

        if points is None:
            rospy.loginfo_throttle(
                3.0, "Waiting for LiDAR topic: %s", self.point_cloud_topic
            )
            return

        for name in self.CAMERA_NAMES:
            frame = frames[name]
            if frame is None:
                rospy.loginfo_throttle(
                    3.0, "Waiting for camera topic: %s", self.cameras[name]["topic"]
                )
                continue

            expected_size = (
                int(self.cameras[name]["width"]),
                int(self.cameras[name]["height"]),
            )
            actual_size = (frame.shape[1], frame.shape[0])
            if actual_size != expected_size:
                rospy.logerr_throttle(
                    3.0,
                    "%s image size is %sx%s, expected %sx%s",
                    name,
                    actual_size[0],
                    actual_size[1],
                    expected_size[0],
                    expected_size[1],
                )
                continue

            pixels, depths, _indices = project_lidar_points(
                points,
                self.lidar,
                self.cameras[name],
                correction=self.correction,
                minimum_depth=self.minimum_depth,
                maximum_depth=self.maximum_depth,
            )
            self._draw_points(frame, pixels, depths)

            time_difference = abs(
                (frame_stamps[name] - point_stamp).to_sec()
            )
            stale = (
                self.maximum_time_difference > 0.0
                and time_difference > self.maximum_time_difference
            )
            status_color = (0, 0, 255) if stale else (0, 255, 0)
            status = "{} points | dt={:.3f}s{}".format(
                len(pixels),
                time_difference,
                " STALE" if stale else "",
            )
            cv2.putText(
                frame,
                status,
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                status_color,
                2,
                cv2.LINE_AA,
            )
            correction_text = "dxyz={}  drpy={}".format(
                [round(value, 3) for value in self.correction["translation"]],
                [round(value, 2) for value in self.correction["rotation"]],
            )
            cv2.putText(
                frame,
                correction_text,
                (15, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "{}_lidar_overlay".format(name)
            self.publishers[name].publish(message)

            if self.gui_available:
                try:
                    cv2.imshow("{} LiDAR overlay".format(name), frame)
                except cv2.error as error:
                    rospy.logwarn("OpenCV windows disabled: %s", error)
                    self.gui_available = False

        if self.gui_available:
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                rospy.signal_shutdown("q pressed in calibration window")

    def run(self):
        """Render in the main thread so OpenCV/Qt windows repaint correctly."""
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self._render()
            rate.sleep()

    def shutdown(self):
        if self.gui_available:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass


def main():
    rospy.init_node("lidar_camera_calibration")
    node = LidarCameraCalibrationNode()
    node.run()


if __name__ == "__main__":
    main()
