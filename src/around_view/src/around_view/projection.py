"""Projection helpers derived from the original udp_avm_auto.py."""

import math

import cv2
import numpy as np


def _scaled_polygon(points, image_width, image_height):
    """Convert normalized polygon coordinates to image pixel coordinates."""
    return np.asarray(
        [
            [
                round(float(point[0]) * (image_width - 1)),
                round(float(point[1]) * (image_height - 1)),
            ]
            for point in points
        ],
        dtype=np.int32,
    )


def source_ground_mask(image_width, image_height, mask_config):
    """Build a mask containing only usable ground pixels.

    Polygon coordinates are normalized to the fixed MORAI camera resolution.
    Exclusion polygons remove the ego hood, mirrors, and body before the
    perspective transform can stretch them across the bird's-eye view.
    """
    mask = np.zeros((int(image_height), int(image_width)), dtype=np.uint8)
    include_polygon = mask_config.get(
        "include",
        [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
    )
    cv2.fillPoly(
        mask,
        [_scaled_polygon(include_polygon, image_width, image_height)],
        255,
    )
    for polygon in mask_config.get("exclude", []):
        cv2.fillPoly(
            mask,
            [_scaled_polygon(polygon, image_width, image_height)],
            0,
        )

    erosion_pixels = max(0, int(mask_config.get("erosion_px", 0)))
    if erosion_pixels:
        kernel_size = erosion_pixels * 2 + 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
    return mask


def feather_mask(binary_mask, feather_pixels):
    """Return a floating-point camera weight with a soft seam boundary."""
    weight = binary_mask.astype(np.float32) / 255.0
    feather_pixels = float(feather_pixels)
    if feather_pixels > 0:
        weight = cv2.GaussianBlur(
            weight,
            (0, 0),
            sigmaX=feather_pixels,
            sigmaY=feather_pixels,
            borderType=cv2.BORDER_CONSTANT,
        )
    return np.clip(weight, 0.0, 1.0)


def warp_image_points(image_points, homography):
    """Transform point centres without warping their rendered marker shapes."""
    points = np.asarray(image_points, dtype=np.float32)
    if points.size == 0:
        return np.empty((0, 2), dtype=np.float32)
    points = points.reshape((-1, 1, 2))
    return cv2.perspectiveTransform(
        points, np.asarray(homography, dtype=np.float64)
    ).reshape((-1, 2))


def vehicle_points_to_canvas(vehicle_points, ground_projection):
    """Map MORAI vehicle-frame x/y coordinates to AVM canvas pixels."""
    points = np.asarray(vehicle_points, dtype=np.float64)
    if points.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    points = points.reshape((-1, 3))
    pixels_per_meter = float(ground_projection["pixels_per_meter"])
    ego_center_x, ego_center_y = [
        float(value) for value in ground_projection["ego_center"]
    ]
    return np.column_stack(
        (
            ego_center_x - points[:, 1] * pixels_per_meter,
            ego_center_y - points[:, 0] * pixels_per_meter,
        )
    )


def ground_to_camera_pixel(
    camera_name,
    x_ground,
    y_ground,
    camera_config,
    offsets,
    image_width,
    image_height,
):
    """Project one ground-plane point into a MORAI camera image."""
    camera_x, camera_y, camera_z = [
        float(value) for value in camera_config["position"]
    ]
    _roll, pitch, yaw = [
        float(value) for value in camera_config["rotation"]
    ]
    horizontal_fov = math.radians(float(camera_config["fov"]))

    camera_z += float(offsets.get("global_height_cm", 0)) / 100.0
    if camera_name == "front":
        pitch += float(offsets.get("front_pitch_tenths", 0)) / 10.0
    elif camera_name == "left":
        yaw += float(offsets.get("left_yaw_tenths", 0)) / 10.0
        pitch += float(offsets.get("left_pitch_tenths", 0)) / 10.0
    elif camera_name == "right":
        yaw += float(offsets.get("right_yaw_tenths", 0)) / 10.0
        pitch += float(offsets.get("right_pitch_tenths", 0)) / 10.0

    delta_x = float(x_ground) - camera_x
    delta_y = float(y_ground) - camera_y
    yaw_radians = math.radians(yaw)
    rotated_x = delta_x * math.cos(yaw_radians) + delta_y * math.sin(
        yaw_radians
    )
    rotated_y = -delta_x * math.sin(yaw_radians) + delta_y * math.cos(
        yaw_radians
    )

    pitch_radians = math.radians(pitch)
    camera_axis_x = -rotated_y
    camera_axis_y = camera_z * math.cos(
        pitch_radians
    ) - rotated_x * math.sin(pitch_radians)
    camera_axis_z = rotated_x * math.cos(
        pitch_radians
    ) + camera_z * math.sin(pitch_radians)

    if camera_axis_z <= 0.1:
        return None

    focal_length = (float(image_width) / 2.0) / math.tan(
        horizontal_fov / 2.0
    )
    pixel_x = (
        focal_length * camera_axis_x / camera_axis_z
        + float(image_width) / 2.0
    )
    pixel_y = (
        focal_length * camera_axis_y / camera_axis_z
        + float(image_height) / 2.0
    )
    return [pixel_x, pixel_y]


def auto_homography(
    camera_name,
    camera_config,
    offsets,
    image_width,
    image_height,
    destination_points,
    ground_projection,
):
    """Calculate the image-to-BEV homography used by udp_avm_auto.py."""
    pixels_per_meter = float(ground_projection["pixels_per_meter"])
    ego_center_x, ego_center_y = [
        float(value) for value in ground_projection["ego_center"]
    ]

    source_points = []
    valid_destinations = []
    for destination in destination_points:
        destination_x, destination_y = [
            float(value) for value in destination
        ]
        x_ground = (ego_center_y - destination_y) / pixels_per_meter
        y_ground = (ego_center_x - destination_x) / pixels_per_meter
        camera_pixel = ground_to_camera_pixel(
            camera_name,
            x_ground,
            y_ground,
            camera_config,
            offsets,
            image_width,
            image_height,
        )
        if camera_pixel is not None:
            source_points.append(camera_pixel)
            valid_destinations.append([destination_x, destination_y])

    if len(source_points) != 4:
        return None

    return cv2.getPerspectiveTransform(
        np.float32(source_points), np.float32(valid_destinations)
    )
