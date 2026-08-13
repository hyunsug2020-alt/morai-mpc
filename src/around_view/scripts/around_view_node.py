#!/usr/bin/env python3
"""Compose front, left, and right ROS camera images into an AVM image."""

import os
import threading

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import yaml
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2

from around_view import (
    auto_homography,
    build_object_contour_mask,
    feather_mask,
    project_lidar_points,
    source_ground_mask,
    transform_lidar_points_to_vehicle,
    warp_image_points,
)


DEFAULT_CAMERAS = {
    "front": {"topic": "/around_view/camera/front/image_raw"},
    "left": {"topic": "/around_view/camera/left/image_raw"},
    "right": {"topic": "/around_view/camera/right/image_raw"},
}

DEFAULT_CALIBRATION = {
    "front": {"y_top": 400, "x_top": 150, "y_bottom": 680, "x_bottom": 580},
    "left": {"y_top": 400, "x_top": 150, "y_bottom": 680, "x_bottom": 580},
    "right": {"y_top": 400, "x_top": 150, "y_bottom": 680, "x_bottom": 580},
}

DEFAULT_DESTINATIONS = {
    "front": [[280, 50], [520, 50], [520, 290], [280, 290]],
    "left": [[160, 50], [340, 50], [340, 490], [160, 490]],
    "right": [[460, 50], [640, 50], [640, 490], [460, 490]],
}

DEFAULT_CAMERA_GEOMETRY = {
    "front": {
        "position": [1.9, 0.0, 1.2],
        "rotation": [0.0, 2.0, 0.0],
        "fov": 90.0,
    },
    "left": {
        "position": [1.15, 0.65, 1.2],
        "rotation": [0.0, 10.0, 70.0],
        "fov": 130.0,
    },
    "right": {
        "position": [1.15, -0.65, 1.2],
        "rotation": [0.0, 10.0, 290.0],
        "fov": 130.0,
    },
}

DEFAULT_AUTO_OFFSETS = {
    "front_pitch_tenths": 0,
    "left_yaw_tenths": 0,
    "right_yaw_tenths": 0,
    "left_pitch_tenths": 0,
    "right_pitch_tenths": 0,
    "global_height_cm": 0,
}

DEFAULT_SEAM_POLYGONS = {
    "front": [
        [-50, 0],
        [850, 0],
        [500, 350],
        [436, 358],
        [364, 358],
        [300, 350],
    ],
    "left": [[0, 0], [400, 0], [400, 500], [0, 500]],
    "right": [[400, 0], [800, 0], [800, 500], [400, 500]],
}

DEFAULT_SOURCE_GROUND_MASKS = {
    "front": {
        "include": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
        "exclude": [],
        "erosion_px": 0,
    },
    "left": {
        "include": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
        "exclude": [],
        "erosion_px": 0,
    },
    "right": {
        "include": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
        "exclude": [],
        "erosion_px": 0,
    },
}


class AroundViewNode:
    CAMERA_NAMES = ("front", "left", "right")

    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = {name: None for name in self.CAMERA_NAMES}
        self.frame_stamps = {name: rospy.Time(0) for name in self.CAMERA_NAMES}
        self.lidar_points = None
        self.lidar_stamp = rospy.Time(0)

        self.cameras = rospy.get_param("~cameras", DEFAULT_CAMERAS)
        self.projection_mode = rospy.get_param(
            "~projection_mode", "auto_geometry"
        )
        self.calibration = rospy.get_param("~calibration", DEFAULT_CALIBRATION)
        self.destinations = rospy.get_param(
            "~destinations", DEFAULT_DESTINATIONS
        )
        self.camera_geometry = rospy.get_param(
            "~camera_geometry", DEFAULT_CAMERA_GEOMETRY
        )
        self.auto_offsets = rospy.get_param(
            "~auto_offsets", DEFAULT_AUTO_OFFSETS
        )
        self.ground_projection = rospy.get_param(
            "~ground_projection",
            {"pixels_per_meter": 40.0, "ego_center": [400, 450]},
        )
        self.seam_polygons = rospy.get_param(
            "~seam_polygons", DEFAULT_SEAM_POLYGONS
        )
        self.seam_feather_pixels = float(
            rospy.get_param("~seam_feather_pixels", 0.0)
        )
        self.source_ground_masks = rospy.get_param(
            "~source_ground_masks", DEFAULT_SOURCE_GROUND_MASKS
        )
        canvas = rospy.get_param("~canvas", {"width": 800, "height": 500})
        self.canvas_width = int(canvas.get("width", 800))
        self.canvas_height = int(canvas.get("height", 500))
        self.vehicle = rospy.get_param(
            "~vehicle",
            {
                "body": [364, 358, 436, 500],
                "windshield": [374, 375, 426, 395],
                "label_position": [385, 450],
                "label_scale": 0.45,
                "line_thickness": 1,
            },
        )

        self.output_topic = rospy.get_param("~output_topic", "/around_view/image")
        self.output_frame_id = rospy.get_param(
            "~output_frame_id", "around_view"
        )
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.show_window = bool(rospy.get_param("~show_window", True))
        self.calibration_file = os.path.expanduser(
            rospy.get_param(
                "~calibration_file",
                "~/.ros/around_view/calibration.yaml",
            )
        )
        lidar_overlay = rospy.get_param("~lidar_overlay", {})
        self.lidar_overlay_enabled = bool(
            lidar_overlay.get("enabled", False)
        )
        self.point_cloud_topic = lidar_overlay.get(
            "point_cloud_topic", "/velodyne_points"
        )
        self.lidar = lidar_overlay.get("lidar", {})
        self.lidar_cameras = lidar_overlay.get("cameras", {})
        self.lidar_correction = lidar_overlay.get(
            "lidar_correction",
            {
                "translation": [0.0, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0],
            },
        )
        lidar_projection = lidar_overlay.get("projection", {})
        self.lidar_minimum_depth = float(
            lidar_projection.get("minimum_depth", 0.5)
        )
        self.lidar_maximum_depth = float(
            lidar_projection.get("maximum_depth", 80.0)
        )
        self.lidar_point_stride = max(
            1, int(lidar_projection.get("point_stride", 1))
        )
        self.lidar_maximum_points = max(
            100, int(lidar_projection.get("maximum_points", 30000))
        )
        self.lidar_point_radius = max(
            1, int(lidar_projection.get("point_radius", 2))
        )
        self.lidar_maximum_time_difference = max(
            0.0,
            float(
                lidar_projection.get("maximum_time_difference", 0.30)
            ),
        )
        self.lidar_render_mode = str(
            lidar_projection.get("render_mode", "camera_points")
        ).lower()
        self.lidar_minimum_object_height = float(
            lidar_projection.get("minimum_object_height", 0.15)
        )
        self.lidar_maximum_object_height = float(
            lidar_projection.get("maximum_object_height", 2.80)
        )
        self.lidar_minimum_planar_range = max(
            0.0,
            float(lidar_projection.get("minimum_planar_range", 0.8)),
        )
        self.lidar_maximum_planar_range = max(
            self.lidar_minimum_planar_range,
            float(lidar_projection.get("maximum_planar_range", 25.0)),
        )
        self.lidar_outlier_radius = max(
            0.01,
            float(lidar_projection.get("outlier_radius", 0.20)),
        )
        self.lidar_outlier_minimum_neighbors = max(
            1,
            int(lidar_projection.get("outlier_minimum_neighbors", 3)),
        )
        self.lidar_cluster_connection = max(
            0.01,
            float(lidar_projection.get("cluster_connection", 0.20)),
        )
        self.lidar_cluster_minimum_points = max(
            1,
            int(lidar_projection.get("cluster_minimum_points", 6)),
        )
        self.lidar_contour_minimum_area = max(
            0.0,
            float(lidar_projection.get("contour_minimum_area", 8.0)),
        )
        self.lidar_contour_minimum_length = max(
            0.0,
            float(lidar_projection.get("contour_minimum_length", 12.0)),
        )
        self.lidar_contour_thickness = max(
            1, int(lidar_projection.get("contour_thickness", 2))
        )
        self.lidar_fit_vehicle_boxes = bool(
            lidar_projection.get("fit_vehicle_boxes", True)
        )
        self.lidar_vehicle_minimum_points = max(
            1,
            int(lidar_projection.get("vehicle_minimum_points", 18)),
        )
        self.lidar_vehicle_minimum_height_span = max(
            0.0,
            float(
                lidar_projection.get(
                    "vehicle_minimum_height_span",
                    0.45,
                )
            ),
        )
        self.lidar_vehicle_observed_minimum_length = max(
            0.0,
            float(
                lidar_projection.get(
                    "vehicle_observed_minimum_length",
                    1.2,
                )
            ),
        )
        self.lidar_vehicle_observed_maximum_length = max(
            self.lidar_vehicle_observed_minimum_length,
            float(
                lidar_projection.get(
                    "vehicle_observed_maximum_length",
                    6.5,
                )
            ),
        )
        self.lidar_vehicle_maximum_lateral_distance = max(
            0.0,
            float(
                lidar_projection.get(
                    "vehicle_maximum_lateral_distance",
                    2.5,
                )
            ),
        )
        self.lidar_vehicle_length = max(
            0.1,
            float(lidar_projection.get("vehicle_length", 4.0)),
        )
        self.lidar_vehicle_width = max(
            0.1,
            float(lidar_projection.get("vehicle_width", 1.7)),
        )
        self.lidar_vehicle_radial_alignment_threshold = float(
            np.clip(
                lidar_projection.get(
                    "vehicle_radial_alignment_threshold",
                    0.65,
                ),
                0.0,
                1.0,
            )
        )
        self.lidar_contour_smoothing_alpha = float(
            np.clip(
                lidar_projection.get("contour_smoothing_alpha", 1.0),
                0.01,
                1.0,
            )
        )
        self.lidar_contour_opacity = float(
            np.clip(
                lidar_projection.get("contour_opacity", 0.90),
                0.0,
                1.0,
            )
        )
        contour_color = lidar_projection.get(
            "contour_color_bgr", [0, 165, 255]
        )
        self.lidar_contour_color = tuple(
            int(np.clip(value, 0, 255)) for value in contour_color
        )
        self.lidar_contour_count = 0
        self.lidar_contour_point_count = 0
        self.lidar_contour_target = None
        self.lidar_contour_display = None
        self.lidar_contour_target_stamp = None
        self.lidar_calibration_file = os.path.expanduser(
            lidar_overlay.get(
                "calibration_file",
                "~/.ros/around_view/lidar_camera_calibration.yaml",
            )
        )
        self.gui_available = self.show_window

        self._load_saved_calibration()
        self._load_saved_lidar_correction()
        self._validate_configuration()
        self.seam_weights = self._build_seam_weights()
        self.projection_cache = {}

        self.publisher = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.subscribers = []
        for name in self.CAMERA_NAMES:
            topic = self.cameras[name]["topic"]
            self.subscribers.append(
                rospy.Subscriber(
                    topic,
                    Image,
                    self._image_callback,
                    callback_args=name,
                    queue_size=1,
                    buff_size=2**24,
                )
            )
        self.point_subscriber = None
        if self.lidar_overlay_enabled:
            self.point_subscriber = rospy.Subscriber(
                self.point_cloud_topic,
                PointCloud2,
                self._point_cloud_callback,
                queue_size=1,
                buff_size=2**24,
            )

        rospy.on_shutdown(self.shutdown)

    def _validate_configuration(self):
        if self.projection_mode not in ("auto_geometry", "manual"):
            raise ValueError(
                "projection_mode must be auto_geometry or manual"
            )
        for name in self.CAMERA_NAMES:
            if name not in self.cameras:
                raise ValueError("Missing camera config: {}".format(name))
            if self.projection_mode == "manual" and name not in self.calibration:
                raise ValueError("Missing calibration config: {}".format(name))
            if (
                self.projection_mode == "auto_geometry"
                and name not in self.camera_geometry
            ):
                raise ValueError(
                    "Missing camera geometry config: {}".format(name)
                )
            if name not in self.destinations:
                raise ValueError("Missing destination config: {}".format(name))
            if len(self.destinations[name]) != 4:
                raise ValueError(
                    "{} destination must contain four points".format(name)
                )
        if not self.lidar_overlay_enabled:
            return
        if self.lidar_render_mode not in ("camera_points", "contours"):
            raise ValueError(
                "lidar_overlay/projection/render_mode must be "
                "camera_points or contours"
            )
        for key in ("position", "rotation"):
            if key not in self.lidar or len(self.lidar[key]) != 3:
                raise ValueError(
                    "lidar_overlay/lidar/{} must contain three values".format(
                        key
                    )
                )
        for name in self.CAMERA_NAMES:
            if name not in self.lidar_cameras:
                raise ValueError(
                    "Missing lidar_overlay/cameras/{}".format(name)
                )
            for key in ("position", "rotation", "width", "height", "fov"):
                if key not in self.lidar_cameras[name]:
                    raise ValueError(
                        "Missing lidar_overlay/cameras/{}/{}".format(
                            name, key
                        )
                    )

    def _build_seam_weights(self):
        if self.projection_mode != "auto_geometry":
            return {}

        weights = {}
        for name in self.CAMERA_NAMES:
            polygon = self.seam_polygons.get(name)
            if not polygon:
                continue
            mask = np.zeros(
                (self.canvas_height, self.canvas_width), dtype=np.uint8
            )
            cv2.fillPoly(mask, [np.asarray(polygon, dtype=np.int32)], 255)
            weights[name] = feather_mask(mask, self.seam_feather_pixels)
        return weights

    def _load_saved_calibration(self):
        if not os.path.isfile(self.calibration_file):
            return
        try:
            with open(self.calibration_file, "r") as config_file:
                saved = yaml.safe_load(config_file) or {}
            for name, values in saved.get("calibration", {}).items():
                if name in self.calibration and isinstance(values, dict):
                    self.calibration[name].update(values)
            saved_offsets = saved.get("auto_offsets", {})
            if isinstance(saved_offsets, dict):
                self.auto_offsets.update(saved_offsets)
            rospy.loginfo("Loaded AVM calibration: %s", self.calibration_file)
        except (OSError, yaml.YAMLError) as error:
            rospy.logwarn("Could not load AVM calibration: %s", error)

    def _load_saved_lidar_correction(self):
        if (
            not self.lidar_overlay_enabled
            or not os.path.isfile(self.lidar_calibration_file)
        ):
            return
        try:
            with open(self.lidar_calibration_file, "r") as config_file:
                saved = yaml.safe_load(config_file) or {}
            correction = saved.get("lidar_correction", {})
            for key in ("translation", "rotation"):
                values = correction.get(key)
                if isinstance(values, list) and len(values) == 3:
                    self.lidar_correction[key] = [
                        float(value) for value in values
                    ]
            rospy.loginfo(
                "Loaded LiDAR-camera correction: %s",
                self.lidar_calibration_file,
            )
        except (OSError, ValueError, yaml.YAMLError) as error:
            rospy.logwarn(
                "Could not load LiDAR-camera correction: %s", error
            )

    def _image_callback(self, message, camera_name):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(3.0, "Image conversion failed: {}".format(error))
            return

        with self.lock:
            self.frames[camera_name] = frame
            self.frame_stamps[camera_name] = message.header.stamp

    def _point_cloud_callback(self, message):
        points = []
        for index, point in enumerate(
            point_cloud2.read_points(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        ):
            if index % self.lidar_point_stride:
                continue
            points.append(point)
            if len(points) >= self.lidar_maximum_points:
                break

        if not points:
            rospy.logwarn_throttle(
                3.0, "No finite points in {}".format(self.point_cloud_topic)
            )
            return

        stamp = message.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
        with self.lock:
            self.lidar_points = np.asarray(points, dtype=np.float32)
            self.lidar_stamp = stamp

    def _setup_windows(self):
        if not self.gui_available:
            return

        try:
            cv2.namedWindow("Around View Monitor (AVM)", cv2.WINDOW_NORMAL)
        except cv2.error as error:
            rospy.logwarn("OpenCV windows disabled: %s", error)
            self.gui_available = False

    def _homography(self, camera_name, frame):
        image_height, image_width = frame.shape[:2]
        if self.projection_mode == "auto_geometry":
            return auto_homography(
                camera_name,
                self.camera_geometry[camera_name],
                self.auto_offsets,
                image_width,
                image_height,
                self.destinations[camera_name],
                self.ground_projection,
            )

        values = self.calibration[camera_name]
        center_x = image_width / 2.0

        source = np.float32(
            [
                [center_x - values["x_top"], values["y_top"]],
                [center_x + values["x_top"], values["y_top"]],
                [center_x + values["x_bottom"], values["y_bottom"]],
                [center_x - values["x_bottom"], values["y_bottom"]],
            ]
        )
        destination = np.float32(self.destinations[camera_name])
        return cv2.getPerspectiveTransform(source, destination)

    def _projection_data(self, camera_name, frame):
        image_height, image_width = frame.shape[:2]
        if self.projection_mode == "auto_geometry":
            calibration_state = tuple(
                sorted(
                    (key, float(value))
                    for key, value in self.auto_offsets.items()
                )
            )
        else:
            calibration_state = tuple(
                sorted(
                    (key, float(value))
                    for key, value in self.calibration[camera_name].items()
                )
            )
        cache_key = (
            self.projection_mode,
            image_width,
            image_height,
            calibration_state,
        )
        cached = self.projection_cache.get(camera_name)
        if cached and cached[0] == cache_key:
            return cached[1], cached[2]

        matrix = self._homography(camera_name, frame)
        if matrix is None:
            return None, None

        if self.projection_mode == "auto_geometry":
            mask_config = self.source_ground_masks.get(camera_name)
        else:
            mask_config = None
        if mask_config:
            source_mask = source_ground_mask(
                image_width, image_height, mask_config
            )
        else:
            source_mask = np.full(
                (image_height, image_width), 255, dtype=np.uint8
            )
        warped_source_mask = cv2.warpPerspective(
            source_mask,
            matrix,
            (self.canvas_width, self.canvas_height),
            flags=cv2.INTER_NEAREST,
        )
        camera_weight = warped_source_mask.astype(np.float32) / 255.0
        if camera_name in self.seam_weights:
            camera_weight *= self.seam_weights[camera_name]

        self.projection_cache[camera_name] = (
            cache_key,
            matrix,
            camera_weight,
        )
        return matrix, camera_weight

    def _draw_lidar_overlay(
        self,
        canvas,
        points,
        projection_matrices,
        camera_owners,
        lidar_stamp,
    ):
        """Draw fixed-size LiDAR markers after the camera image warp."""
        if not self.lidar_overlay_enabled or points is None:
            return 0
        if self.lidar_render_mode == "contours":
            return self._draw_lidar_contours(canvas, points, lidar_stamp)

        point_count = 0
        self.lidar_contour_count = 0
        color_denominator = max(
            1e-6,
            self.lidar_maximum_depth - self.lidar_minimum_depth,
        )
        for camera_index, name in enumerate(self.CAMERA_NAMES):
            matrix = projection_matrices.get(name)
            if matrix is None:
                continue

            pixels, depths, _indices = project_lidar_points(
                points,
                self.lidar,
                self.lidar_cameras[name],
                correction=self.lidar_correction,
                minimum_depth=self.lidar_minimum_depth,
                maximum_depth=self.lidar_maximum_depth,
            )
            if not len(pixels):
                continue

            canvas_points = warp_image_points(pixels, matrix)
            rounded = np.rint(canvas_points).astype(np.int32)
            inside = (
                np.isfinite(canvas_points).all(axis=1)
                & (rounded[:, 0] >= 0)
                & (rounded[:, 0] < self.canvas_width)
                & (rounded[:, 1] >= 0)
                & (rounded[:, 1] < self.canvas_height)
            )
            inside_indices = np.flatnonzero(inside)
            if not inside_indices.size:
                continue

            owned = (
                camera_owners[
                    rounded[inside_indices, 1],
                    rounded[inside_indices, 0],
                ]
                == camera_index
            )
            draw_indices = inside_indices[owned]
            if not draw_indices.size:
                continue

            normalized = np.clip(
                (
                    depths[draw_indices] - self.lidar_minimum_depth
                )
                / color_denominator,
                0.0,
                1.0,
            )
            color_values = np.asarray(
                np.round((1.0 - normalized) * 255.0),
                dtype=np.uint8,
            )
            colors = cv2.applyColorMap(
                color_values.reshape((-1, 1)),
                cv2.COLORMAP_TURBO,
            ).reshape((-1, 3))

            # The marker is drawn only after its centre has been transformed.
            # This prevents perspective warping from turning it into a blob.
            for order_index in np.argsort(depths[draw_indices])[::-1]:
                point_index = draw_indices[order_index]
                pixel = rounded[point_index]
                color = tuple(int(value) for value in colors[order_index])
                cv2.circle(
                    canvas,
                    (int(pixel[0]), int(pixel[1])),
                    self.lidar_point_radius,
                    color,
                    -1,
                    lineType=cv2.LINE_AA,
                )
            point_count += len(draw_indices)

        return point_count

    def _draw_lidar_contours(self, canvas, points, lidar_stamp):
        """Draw filtered, clustered and temporally smoothed object outlines."""
        vehicle_points = transform_lidar_points_to_vehicle(
            points,
            self.lidar,
            correction=self.lidar_correction,
        )
        stamp_key = (
            None
            if lidar_stamp is None
            else (int(lidar_stamp.secs), int(lidar_stamp.nsecs))
        )
        if (
            self.lidar_contour_target is None
            or stamp_key != self.lidar_contour_target_stamp
        ):
            (
                contour_mask,
                self.lidar_contour_count,
                self.lidar_contour_point_count,
            ) = build_object_contour_mask(
                vehicle_points,
                self.ground_projection,
                (self.canvas_height, self.canvas_width),
                minimum_range=self.lidar_minimum_planar_range,
                maximum_range=self.lidar_maximum_planar_range,
                minimum_height=self.lidar_minimum_object_height,
                maximum_height=self.lidar_maximum_object_height,
                outlier_radius=self.lidar_outlier_radius,
                outlier_minimum_neighbors=(
                    self.lidar_outlier_minimum_neighbors
                ),
                cluster_connection=self.lidar_cluster_connection,
                cluster_minimum_points=self.lidar_cluster_minimum_points,
                contour_minimum_area=self.lidar_contour_minimum_area,
                contour_minimum_length=self.lidar_contour_minimum_length,
                contour_thickness=self.lidar_contour_thickness,
                fit_vehicle_boxes=self.lidar_fit_vehicle_boxes,
                vehicle_minimum_points=self.lidar_vehicle_minimum_points,
                vehicle_minimum_height_span=(
                    self.lidar_vehicle_minimum_height_span
                ),
                vehicle_observed_minimum_length=(
                    self.lidar_vehicle_observed_minimum_length
                ),
                vehicle_observed_maximum_length=(
                    self.lidar_vehicle_observed_maximum_length
                ),
                vehicle_maximum_lateral_distance=(
                    self.lidar_vehicle_maximum_lateral_distance
                ),
                vehicle_length=self.lidar_vehicle_length,
                vehicle_width=self.lidar_vehicle_width,
                vehicle_radial_alignment_threshold=(
                    self.lidar_vehicle_radial_alignment_threshold
                ),
            )
            self.lidar_contour_target = (
                contour_mask.astype(np.float32) / 255.0
            )
            self.lidar_contour_target_stamp = stamp_key
            if self.lidar_contour_display is None:
                self.lidar_contour_display = (
                    self.lidar_contour_target.copy()
                )

        alpha = self.lidar_contour_smoothing_alpha
        self.lidar_contour_display += alpha * (
            self.lidar_contour_target - self.lidar_contour_display
        )
        overlay_alpha = np.clip(
            self.lidar_contour_display * self.lidar_contour_opacity,
            0.0,
            1.0,
        )[:, :, None]
        contour_color = np.asarray(
            self.lidar_contour_color,
            dtype=np.float32,
        ).reshape((1, 1, 3))
        canvas[:] = np.clip(
            canvas.astype(np.float32) * (1.0 - overlay_alpha)
            + contour_color * overlay_alpha,
            0.0,
            255.0,
        ).astype(np.uint8)
        return self.lidar_contour_point_count

    def _compose(self):
        with self.lock:
            frames = {
                name: None if frame is None else frame.copy()
                for name, frame in self.frames.items()
            }
            stamps = dict(self.frame_stamps)
            lidar_points = (
                None
                if self.lidar_points is None
                else self.lidar_points.copy()
            )
            lidar_stamp = self.lidar_stamp

        accumulator = np.zeros(
            (self.canvas_height, self.canvas_width, 3), dtype=np.float32
        )
        accumulated_weight = np.zeros(
            (self.canvas_height, self.canvas_width), dtype=np.float32
        )
        camera_owners = np.full(
            (self.canvas_height, self.canvas_width), -1, dtype=np.int8
        )
        dominant_weight = np.zeros(
            (self.canvas_height, self.canvas_width), dtype=np.float32
        )
        projection_matrices = {}
        available = []

        for camera_index, name in enumerate(self.CAMERA_NAMES):
            frame = frames[name]
            if frame is None:
                continue
            available.append(name)
            matrix, camera_weight = self._projection_data(name, frame)
            if matrix is None:
                rospy.logwarn_throttle(
                    3.0,
                    "Could not calculate {} camera homography".format(name),
                )
                continue
            projection_matrices[name] = matrix
            warped = cv2.warpPerspective(
                frame, matrix, (self.canvas_width, self.canvas_height)
            )

            if self.seam_feather_pixels <= 0:
                use_camera = np.logical_and(
                    camera_weight > 0.5,
                    accumulated_weight <= 1e-4,
                )
                accumulator[use_camera] = warped[use_camera]
                accumulated_weight[use_camera] = 1.0
                camera_owners[use_camera] = camera_index
            else:
                accumulator += (
                    warped.astype(np.float32) * camera_weight[:, :, None]
                )
                accumulated_weight += camera_weight
                use_camera = camera_weight > dominant_weight
                camera_owners[use_camera] = camera_index
                dominant_weight[use_camera] = camera_weight[use_camera]

        canvas = np.zeros(
            (self.canvas_height, self.canvas_width, 3), dtype=np.uint8
        )
        valid = accumulated_weight > 1e-4
        if np.any(valid):
            canvas[valid] = np.clip(
                accumulator[valid] / accumulated_weight[valid, None],
                0,
                255,
            ).astype(np.uint8)

        lidar_point_count = self._draw_lidar_overlay(
            canvas,
            lidar_points,
            projection_matrices,
            camera_owners,
            lidar_stamp,
        )

        body = [int(value) for value in self.vehicle["body"]]
        windshield = [int(value) for value in self.vehicle["windshield"]]
        label_position = tuple(
            int(value) for value in self.vehicle["label_position"]
        )
        label_scale = float(self.vehicle.get("label_scale", 0.45))
        line_thickness = int(self.vehicle.get("line_thickness", 1))
        blind_zone_polygon = self.vehicle.get("blind_zone_polygon")
        if blind_zone_polygon:
            blind_polygon = np.asarray(blind_zone_polygon, dtype=np.int32)
            blind_mask = np.zeros(
                (self.canvas_height, self.canvas_width), dtype=np.uint8
            )
            cv2.fillPoly(blind_mask, [blind_polygon], 255)
            cv2.fillPoly(canvas, [blind_polygon], (38, 38, 38))

            hatch = np.zeros_like(canvas)
            for offset in range(
                -self.canvas_height,
                self.canvas_width + self.canvas_height,
                18,
            ):
                cv2.line(
                    hatch,
                    (offset, self.canvas_height),
                    (offset + self.canvas_height, 0),
                    (62, 62, 62),
                    1,
                )
            canvas[blind_mask > 0] = np.maximum(
                canvas[blind_mask > 0],
                hatch[blind_mask > 0],
            )
            cv2.polylines(
                canvas,
                [blind_polygon],
                True,
                (105, 105, 105),
                line_thickness,
            )
        occlusion_polygon = self.vehicle.get("occlusion_polygon")
        if occlusion_polygon:
            cv2.fillPoly(
                canvas,
                [np.asarray(occlusion_polygon, dtype=np.int32)],
                (20, 20, 20),
            )
        cv2.rectangle(
            canvas, (body[0], body[1]), (body[2], body[3]), (70, 70, 70), -1
        )
        cv2.rectangle(
            canvas,
            (body[0], body[1]),
            (body[2], body[3]),
            (255, 255, 255),
            line_thickness,
        )
        cv2.rectangle(
            canvas,
            (windshield[0], windshield[1]),
            (windshield[2], windshield[3]),
            (200, 200, 200),
            -1,
        )
        cv2.putText(
            canvas,
            "EGO",
            label_position,
            cv2.FONT_HERSHEY_SIMPLEX,
            label_scale,
            (255, 255, 255),
            line_thickness,
        )

        valid_stamps = [
            stamps[name] for name in available if stamps[name] != rospy.Time(0)
        ]
        stamp = max(valid_stamps) if valid_stamps else rospy.Time.now()
        if self.lidar_overlay_enabled:
            if lidar_points is None:
                status = "Waiting for {}".format(self.point_cloud_topic)
                status_color = (0, 165, 255)
            else:
                time_difference = abs((stamp - lidar_stamp).to_sec())
                stale = (
                    self.lidar_maximum_time_difference > 0.0
                    and time_difference > self.lidar_maximum_time_difference
                )
                if self.lidar_render_mode == "contours":
                    status = (
                        "LiDAR {} contours ({} pts) | dt={:.3f}s{}"
                    ).format(
                        self.lidar_contour_count,
                        lidar_point_count,
                        time_difference,
                        " STALE" if stale else "",
                    )
                else:
                    status = "LiDAR {} pts | dt={:.3f}s{}".format(
                        lidar_point_count,
                        time_difference,
                        " STALE" if stale else "",
                    )
                status_color = (0, 0, 255) if stale else (0, 255, 0)
            cv2.putText(
                canvas,
                status,
                (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                status_color,
                1,
                cv2.LINE_AA,
            )
        return canvas, available, stamp

    def shutdown(self):
        if self.gui_available:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass

    def run(self):
        self._setup_windows()
        rate = rospy.Rate(self.publish_rate)
        rospy.loginfo(
            "Around-view node publishing %s with %s projection",
            self.output_topic,
            self.projection_mode,
        )

        while not rospy.is_shutdown():
            canvas, available, stamp = self._compose()

            if available:
                message = self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
                message.header.stamp = stamp
                message.header.frame_id = self.output_frame_id
                self.publisher.publish(message)

            if len(available) != len(self.CAMERA_NAMES):
                waiting = sorted(set(self.CAMERA_NAMES) - set(available))
                rospy.loginfo_throttle(
                    3.0,
                    "Waiting for ROS camera topics: {}".format(
                        ", ".join(waiting)
                    ),
                )

            if self.gui_available:
                cv2.imshow("Around View Monitor (AVM)", canvas)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    rospy.signal_shutdown("q pressed in AVM window")
                    break

            rate.sleep()


def main():
    rospy.init_node("around_view")
    node = AroundViewNode()
    node.run()


if __name__ == "__main__":
    main()
