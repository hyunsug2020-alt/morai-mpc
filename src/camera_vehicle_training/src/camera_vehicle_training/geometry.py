"""Geometry used to turn MORAI object truth into camera YOLO boxes."""

import math

import numpy as np


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def morai_rotation_matrix(rotation_degrees):
    """Return the sensor-to-vehicle rotation for MORAI roll/pitch/yaw."""
    roll, pitch, yaw = [
        math.radians(float(value)) for value in rotation_degrees
    ]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    roll_matrix = np.array(
        [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]],
        dtype=np.float64,
    )
    # MORAI pitch is positive when a camera looks down.
    pitch_matrix = np.array(
        [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]],
        dtype=np.float64,
    )
    yaw_matrix = np.array(
        [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return yaw_matrix @ pitch_matrix @ roll_matrix


def camera_intrinsic_matrix(width, height, horizontal_fov_degrees):
    focal = (float(width) / 2.0) / math.tan(
        math.radians(float(horizontal_fov_degrees)) / 2.0
    )
    return np.array(
        [
            [focal, 0.0, float(width) / 2.0],
            [0.0, focal, float(height) / 2.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def world_object_to_vehicle(
    object_position,
    object_heading_degrees,
    object_size,
    ego_position,
    ego_heading_degrees,
    position_is_bottom=True,
):
    """Convert one MORAI world-frame object pose to the vehicle frame."""
    object_position = np.asarray(object_position, dtype=np.float64)
    ego_position = np.asarray(ego_position, dtype=np.float64)
    size = np.asarray(object_size, dtype=np.float64)
    delta = object_position - ego_position
    ego_yaw = math.radians(float(ego_heading_degrees))
    cosine, sine = math.cos(ego_yaw), math.sin(ego_yaw)
    center = np.array(
        [
            cosine * delta[0] + sine * delta[1],
            -sine * delta[0] + cosine * delta[1],
            delta[2],
        ],
        dtype=np.float64,
    )
    if position_is_bottom:
        center[2] += 0.5 * size[2]
    yaw = normalize_angle(
        math.radians(float(object_heading_degrees)) - ego_yaw
    )
    return center, size, yaw


def make_vehicle_box_corners(center, size, yaw):
    """Return the eight corners of a z-up oriented 3D box."""
    center = np.asarray(center, dtype=np.float64)
    half = 0.5 * np.asarray(size, dtype=np.float64)
    cosine, sine = math.cos(float(yaw)), math.sin(float(yaw))
    corners = []
    for z_sign in (-1.0, 1.0):
        for x_sign, y_sign in (
            (-1.0, -1.0),
            (1.0, -1.0),
            (1.0, 1.0),
            (-1.0, 1.0),
        ):
            local_x = x_sign * half[0]
            local_y = y_sign * half[1]
            corners.append(
                [
                    center[0] + cosine * local_x - sine * local_y,
                    center[1] + sine * local_x + cosine * local_y,
                    center[2] + z_sign * half[2],
                ]
            )
    return np.asarray(corners, dtype=np.float64)


def project_vehicle_points(points, camera):
    """Project vehicle-frame points to pixels and return pixels/depth/mask."""
    points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    translation = np.asarray(camera["position"], dtype=np.float64)
    rotation = morai_rotation_matrix(camera["rotation"])
    body = (points - translation) @ rotation
    optical = np.column_stack((-body[:, 1], -body[:, 2], body[:, 0]))
    depth = optical[:, 2]
    minimum_depth = float(camera.get("minimum_depth", 0.5))
    valid = np.isfinite(depth) & (depth > minimum_depth)
    pixels = np.full((len(points), 2), np.nan, dtype=np.float64)
    if np.any(valid):
        intrinsic = camera_intrinsic_matrix(
            camera["width"], camera["height"], camera["fov"]
        )
        pixels[valid, 0] = (
            intrinsic[0, 0] * optical[valid, 0] / depth[valid]
            + intrinsic[0, 2]
        )
        pixels[valid, 1] = (
            intrinsic[1, 1] * optical[valid, 1] / depth[valid]
            + intrinsic[1, 2]
        )
    return pixels, depth, valid


def projected_box(corners, camera, minimum_inside_ratio=0.30):
    """Return a clipped xyxy box and its pre-clip inside-area ratio."""
    pixels, _depth, valid = project_vehicle_points(corners, camera)
    # A valid car box must lie completely in front of the near plane. This
    # avoids huge boxes when an object crosses through the camera origin.
    if int(np.count_nonzero(valid)) != len(corners):
        return None
    raw_x1 = float(np.min(pixels[:, 0]))
    raw_y1 = float(np.min(pixels[:, 1]))
    raw_x2 = float(np.max(pixels[:, 0]))
    raw_y2 = float(np.max(pixels[:, 1]))
    raw_area = max(0.0, raw_x2 - raw_x1) * max(0.0, raw_y2 - raw_y1)
    if raw_area <= 0.0:
        return None
    width, height = int(camera["width"]), int(camera["height"])
    x1 = min(max(raw_x1, 0.0), float(width - 1))
    y1 = min(max(raw_y1, 0.0), float(height - 1))
    x2 = min(max(raw_x2, 0.0), float(width - 1))
    y2 = min(max(raw_y2, 0.0), float(height - 1))
    clipped_area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    inside_ratio = clipped_area / raw_area
    if clipped_area <= 0.0 or inside_ratio < float(minimum_inside_ratio):
        return None
    return (x1, y1, x2, y2), inside_ratio


def xyxy_to_yolo(box, width, height):
    x1, y1, x2, y2 = (float(value) for value in box)
    return (
        0.5 * (x1 + x2) / float(width),
        0.5 * (y1 + y2) / float(height),
        (x2 - x1) / float(width),
        (y2 - y1) / float(height),
    )
