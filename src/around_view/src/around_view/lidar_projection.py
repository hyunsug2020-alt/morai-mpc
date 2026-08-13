"""MORAI LiDAR-to-camera projection helpers.

MORAI sensor poses use a vehicle frame with x forward, y left and z up.
Camera image coordinates use x right, y down and z forward.
"""

import math

import numpy as np


def morai_rotation_matrix(rotation_degrees):
    """Return the sensor-to-vehicle rotation for MORAI roll/pitch/yaw."""
    roll, pitch, yaw = [
        math.radians(float(value)) for value in rotation_degrees
    ]

    cos_roll, sin_roll = math.cos(roll), math.sin(roll)
    cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

    roll_matrix = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cos_roll, -sin_roll],
            [0.0, sin_roll, cos_roll],
        ],
        dtype=np.float64,
    )
    # MORAI camera pitch is positive when looking down.
    pitch_matrix = np.array(
        [
            [cos_pitch, 0.0, sin_pitch],
            [0.0, 1.0, 0.0],
            [-sin_pitch, 0.0, cos_pitch],
        ],
        dtype=np.float64,
    )
    yaw_matrix = np.array(
        [
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return yaw_matrix @ pitch_matrix @ roll_matrix


def camera_intrinsic_matrix(width, height, horizontal_fov_degrees):
    """Build a pinhole intrinsic matrix from fixed MORAI image settings."""
    width = float(width)
    height = float(height)
    horizontal_fov = math.radians(float(horizontal_fov_degrees))
    focal_length = (width / 2.0) / math.tan(horizontal_fov / 2.0)
    return np.array(
        [
            [focal_length, 0.0, width / 2.0],
            [0.0, focal_length, height / 2.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def transform_lidar_points_to_vehicle(
    lidar_points,
    lidar_config,
    correction=None,
):
    """Transform LiDAR-frame points into the MORAI vehicle frame."""
    points = np.asarray(lidar_points, dtype=np.float64)
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    points = points.reshape((-1, 3))

    correction = correction or {}
    correction_translation = np.asarray(
        correction.get("translation", [0.0, 0.0, 0.0]),
        dtype=np.float64,
    )
    correction_rotation = morai_rotation_matrix(
        correction.get("rotation", [0.0, 0.0, 0.0])
    )

    lidar_translation = np.asarray(
        lidar_config["position"], dtype=np.float64
    )
    lidar_rotation = morai_rotation_matrix(lidar_config["rotation"])
    corrected_lidar_rotation = lidar_rotation @ correction_rotation
    return (
        points @ corrected_lidar_rotation.T
        + lidar_translation
        + correction_translation
    )


def project_lidar_points(
    lidar_points,
    lidar_config,
    camera_config,
    correction=None,
    minimum_depth=0.1,
    maximum_depth=float("inf"),
):
    """Project Nx3 LiDAR points into one MORAI camera image.

    ``correction`` is a small residual adjustment of the LiDAR pose. Its
    translation is expressed in the vehicle frame and its rotation in the
    LiDAR frame. The fixed MORAI sensor poses remain unchanged.
    """
    points = np.asarray(lidar_points, dtype=np.float64)
    if points.size == 0:
        return (
            np.empty((0, 2), dtype=np.float64),
            np.empty((0,), dtype=np.float64),
            np.empty((0,), dtype=np.int64),
        )
    points = points.reshape((-1, 3))

    vehicle_points = transform_lidar_points_to_vehicle(
        points,
        lidar_config,
        correction=correction,
    )

    camera_translation = np.asarray(
        camera_config["position"], dtype=np.float64
    )
    camera_rotation = morai_rotation_matrix(camera_config["rotation"])
    camera_body_points = (
        vehicle_points - camera_translation
    ) @ camera_rotation

    # Camera body axes (forward, left, up) -> optical axes (right, down, front).
    optical_x = -camera_body_points[:, 1]
    optical_y = -camera_body_points[:, 2]
    optical_z = camera_body_points[:, 0]

    width = int(camera_config["width"])
    height = int(camera_config["height"])
    intrinsic = camera_intrinsic_matrix(
        width, height, camera_config["fov"]
    )

    valid_depth = (
        np.isfinite(optical_z)
        & (optical_z > float(minimum_depth))
        & (optical_z < float(maximum_depth))
    )
    indices = np.flatnonzero(valid_depth)
    if not indices.size:
        return (
            np.empty((0, 2), dtype=np.float64),
            np.empty((0,), dtype=np.float64),
            indices,
        )

    depth = optical_z[indices]
    pixel_x = intrinsic[0, 0] * optical_x[indices] / depth + intrinsic[0, 2]
    pixel_y = intrinsic[1, 1] * optical_y[indices] / depth + intrinsic[1, 2]
    inside = (
        np.isfinite(pixel_x)
        & np.isfinite(pixel_y)
        & (pixel_x >= 0.0)
        & (pixel_x < width)
        & (pixel_y >= 0.0)
        & (pixel_y < height)
    )

    return (
        np.column_stack((pixel_x[inside], pixel_y[inside])),
        depth[inside],
        indices[inside],
    )
