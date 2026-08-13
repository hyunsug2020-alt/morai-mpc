"""LiDAR object filtering and contour extraction for the AVM canvas."""

import math

import cv2
import numpy as np

from .projection import vehicle_points_to_canvas


def _odd_size(value):
    size = max(1, int(round(value)))
    return size if size % 2 else size + 1


def _vehicle_box(
    observed_rect,
    ground_projection,
    vehicle_length,
    vehicle_width,
    radial_alignment_threshold,
):
    """Infer a complete vehicle footprint from one visible LiDAR surface."""
    center, (rect_width, rect_height), angle_degrees = observed_rect
    angle = math.radians(float(angle_degrees))
    width_axis = np.array([math.cos(angle), math.sin(angle)])
    height_axis = np.array([-math.sin(angle), math.cos(angle)])

    if rect_width >= rect_height:
        major_axis, minor_axis = width_axis, height_axis
        major_size, minor_size = rect_width, rect_height
    else:
        major_axis, minor_axis = height_axis, width_axis
        major_size, minor_size = rect_height, rect_width

    center_xy = np.asarray(center, dtype=np.float64)
    center_norm = np.linalg.norm(center_xy)
    if center_norm > 1e-6:
        radial_alignment = abs(
            float(np.dot(major_axis, center_xy / center_norm))
        )
    else:
        radial_alignment = 1.0

    if radial_alignment >= float(radial_alignment_threshold):
        length_axis, lateral_axis = major_axis, minor_axis
        observed_length, observed_width = major_size, minor_size
    else:
        # A front/rear face is normally transverse to the LiDAR ray.
        length_axis, lateral_axis = minor_axis, major_axis
        observed_length, observed_width = minor_size, major_size

    fitted_length = max(float(vehicle_length), float(observed_length))
    fitted_width = max(float(vehicle_width), float(observed_width))
    half_length = length_axis * (fitted_length * 0.5)
    half_width = lateral_axis * (fitted_width * 0.5)
    corners_xy = np.asarray(
        [
            center_xy - half_length - half_width,
            center_xy + half_length - half_width,
            center_xy + half_length + half_width,
            center_xy - half_length + half_width,
        ]
    )
    corners = np.column_stack(
        (corners_xy, np.zeros((len(corners_xy),), dtype=np.float64))
    )
    return np.rint(
        vehicle_points_to_canvas(corners, ground_projection)
    ).astype(np.int32)


def build_object_contour_mask(
    vehicle_points,
    ground_projection,
    canvas_shape,
    *,
    minimum_range=0.8,
    maximum_range=25.0,
    minimum_height=0.15,
    maximum_height=2.8,
    outlier_radius=0.20,
    outlier_minimum_neighbors=3,
    cluster_connection=0.20,
    cluster_minimum_points=6,
    contour_minimum_area=8.0,
    contour_minimum_length=12.0,
    contour_thickness=2,
    fit_vehicle_boxes=True,
    vehicle_minimum_points=18,
    vehicle_minimum_height_span=0.45,
    vehicle_observed_minimum_length=1.2,
    vehicle_observed_maximum_length=6.5,
    vehicle_maximum_lateral_distance=2.5,
    vehicle_length=4.0,
    vehicle_width=1.7,
    vehicle_radial_alignment_threshold=0.65,
):
    """Return a filtered uint8 contour mask and extraction statistics.

    Dense points are filtered by range/height, isolated BEV cells are removed,
    and connected components are treated as separate objects. Components that
    resemble a vehicle surface receive a complete oriented footprint.
    """
    points = np.asarray(vehicle_points, dtype=np.float64)
    if points.size == 0:
        return np.zeros(canvas_shape, dtype=np.uint8), 0, 0
    points = points.reshape((-1, 3))

    planar_range = np.linalg.norm(points[:, :2], axis=1)
    valid = (
        np.isfinite(points).all(axis=1)
        & (planar_range >= float(minimum_range))
        & (planar_range <= float(maximum_range))
        & (points[:, 2] >= float(minimum_height))
        & (points[:, 2] <= float(maximum_height))
    )
    points = points[valid]
    if not len(points):
        return np.zeros(canvas_shape, dtype=np.uint8), 0, 0

    height, width = [int(value) for value in canvas_shape]
    canvas_points = vehicle_points_to_canvas(points, ground_projection)
    rounded = np.rint(canvas_points).astype(np.int32)
    inside = (
        np.isfinite(canvas_points).all(axis=1)
        & (rounded[:, 0] >= 0)
        & (rounded[:, 0] < width)
        & (rounded[:, 1] >= 0)
        & (rounded[:, 1] < height)
    )
    points = points[inside]
    rounded = rounded[inside]
    if not len(points):
        return np.zeros(canvas_shape, dtype=np.uint8), 0, 0

    pixels_per_meter = float(ground_projection["pixels_per_meter"])
    point_counts = np.zeros((height, width), dtype=np.float32)
    np.add.at(point_counts, (rounded[:, 1], rounded[:, 0]), 1.0)

    neighbor_size = _odd_size(
        2.0 * float(outlier_radius) * pixels_per_meter + 1.0
    )
    neighbor_counts = cv2.boxFilter(
        point_counts,
        ddepth=-1,
        ksize=(neighbor_size, neighbor_size),
        normalize=False,
        borderType=cv2.BORDER_CONSTANT,
    )
    supported = (
        neighbor_counts[rounded[:, 1], rounded[:, 0]]
        >= float(outlier_minimum_neighbors)
    )
    points = points[supported]
    rounded = rounded[supported]
    if not len(points):
        return np.zeros(canvas_shape, dtype=np.uint8), 0, 0

    seeds = np.zeros((height, width), dtype=np.uint8)
    seeds[rounded[:, 1], rounded[:, 0]] = 255
    connection_size = _odd_size(
        2.0 * float(cluster_connection) * pixels_per_meter + 1.0
    )
    connection_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (connection_size, connection_size),
    )
    connected = cv2.dilate(seeds, connection_kernel)
    component_count, labels, _stats, _centroids = (
        cv2.connectedComponentsWithStats(connected, connectivity=8)
    )

    point_labels = labels[rounded[:, 1], rounded[:, 0]]
    contour_mask = np.zeros((height, width), dtype=np.uint8)
    contour_count = 0

    for label in range(1, component_count):
        member_indices = np.flatnonzero(point_labels == label)
        if len(member_indices) < int(cluster_minimum_points):
            continue

        component_mask = np.zeros((height, width), dtype=np.uint8)
        component_mask[labels == label] = 255
        contours, _hierarchy = cv2.findContours(
            component_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        if (
            cv2.contourArea(contour) < float(contour_minimum_area)
            or cv2.arcLength(contour, True)
            < float(contour_minimum_length)
        ):
            continue

        member_points = points[member_indices]
        observed_rect = cv2.minAreaRect(
            member_points[:, :2].astype(np.float32)
        )
        rect_width, rect_height = observed_rect[1]
        observed_major = max(float(rect_width), float(rect_height))
        height_span = float(np.ptp(member_points[:, 2]))
        cluster_center_y = float(observed_rect[0][1])
        is_vehicle = (
            bool(fit_vehicle_boxes)
            and len(member_points) >= int(vehicle_minimum_points)
            and height_span >= float(vehicle_minimum_height_span)
            and observed_major
            >= float(vehicle_observed_minimum_length)
            and observed_major
            <= float(vehicle_observed_maximum_length)
            and abs(cluster_center_y)
            <= float(vehicle_maximum_lateral_distance)
        )

        if is_vehicle:
            box = _vehicle_box(
                observed_rect,
                ground_projection,
                vehicle_length,
                vehicle_width,
                vehicle_radial_alignment_threshold,
            )
            cv2.polylines(
                contour_mask,
                [box],
                True,
                255,
                int(contour_thickness),
                lineType=cv2.LINE_AA,
            )
        else:
            cv2.drawContours(
                contour_mask,
                [contour],
                -1,
                255,
                int(contour_thickness),
                lineType=cv2.LINE_AA,
            )
        contour_count += 1

    return contour_mask, contour_count, len(points)
