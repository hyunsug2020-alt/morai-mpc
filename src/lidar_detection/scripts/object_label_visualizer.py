#!/usr/bin/env python3
"""Project MORAI ground-truth objects into the live LiDAR coordinate frame."""

import json
import math
import threading
import time
from collections import deque

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf.transformations
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def interpolate_angle_degrees(first, second, ratio):
    delta = math.degrees(
        normalize_angle(math.radians(float(second) - float(first)))
    )
    return float(first) + ratio * delta


class VectorState:
    __slots__ = ("x", "y", "z")

    def __init__(self, x, y, z):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

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
    def interpolate(cls, first, second, target_stamp):
        ratio = interpolation_ratio(first.stamp, second.stamp, target_stamp)
        return cls(
            target_stamp,
            VectorState.interpolate(first.position, second.position, ratio),
            interpolate_angle_degrees(first.heading, second.heading, ratio),
        )


class ObjectState:
    __slots__ = (
        "category",
        "unique_id",
        "name",
        "position",
        "size",
        "heading",
    )

    def __init__(
        self, category, unique_id, name, position, size, heading
    ):
        self.category = category
        self.unique_id = int(unique_id)
        self.name = name
        self.position = position
        self.size = size
        self.heading = float(heading)

    @classmethod
    def from_message(cls, category, message):
        return cls(
            category,
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
            reference.category,
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
        objects = {}
        groups = (
            ("npc", message.npc_list),
            ("pedestrian", message.pedestrian_list),
            ("obstacle", message.obstacle_list),
        )
        for category, items in groups:
            for item in items:
                state = ObjectState.from_message(category, item)
                objects[(category, state.unique_id)] = state
        return cls(message.header.stamp.to_sec(), objects)

    @classmethod
    def interpolate(cls, first, second, target_stamp):
        ratio = interpolation_ratio(first.stamp, second.stamp, target_stamp)
        objects = {}
        all_keys = set(first.objects) | set(second.objects)
        for key in all_keys:
            first_object = first.objects.get(key)
            second_object = second.objects.get(key)
            if first_object is not None and second_object is not None:
                objects[key] = ObjectState.interpolate(
                    first_object, second_object, ratio
                )
            elif ratio <= 0.5 and first_object is not None:
                objects[key] = first_object
            elif ratio > 0.5 and second_object is not None:
                objects[key] = second_object
        return cls(target_stamp, objects)


def interpolation_ratio(first_stamp, second_stamp, target_stamp):
    interval = second_stamp - first_stamp
    if interval <= 1.0e-9:
        return 0.0
    return min(1.0, max(0.0, (target_stamp - first_stamp) / interval))


class GroundTruthVisualizer:
    CATEGORY_COLORS = {
        "npc": (0.10, 0.85, 0.20),
        "pedestrian": (0.95, 0.20, 0.20),
        "obstacle": (1.00, 0.55, 0.05),
    }

    def __init__(self):
        topics = rospy.get_param("~topics")
        lidar = rospy.get_param("~lidar")
        sync = rospy.get_param("~sync", {})
        visualization = rospy.get_param("~visualization", {})

        self.lidar_x = float(lidar["x"])
        self.lidar_y = float(lidar["y"])
        self.lidar_z = float(lidar["z"])
        self.lidar_yaw = math.radians(float(lidar.get("yaw_deg", 0.0)))
        self.position_is_bottom = bool(
            rospy.get_param("~object_position_is_bottom", True)
        )

        self.marker_lifetime = float(
            visualization.get("marker_lifetime_seconds", 0.30)
        )
        self.box_alpha = float(visualization.get("box_alpha", 0.22))
        self.line_width = float(visualization.get("line_width", 0.08))
        self.show_text = bool(visualization.get("show_text", True))
        self.count_points = bool(visualization.get("count_points", True))
        self.padding_xy = float(visualization.get("box_padding_xy", 0.15))
        self.padding_z = float(visualization.get("box_padding_z", 0.15))
        self.buffer_seconds = float(sync.get("buffer_seconds", 1.0))
        self.max_wait_seconds = float(sync.get("max_wait_seconds", 0.15))
        self.max_nearest_seconds = float(
            sync.get("max_nearest_seconds", 0.06)
        )

        self.marker_pub = rospy.Publisher(
            topics["markers"], MarkerArray, queue_size=1
        )
        self.diagnostic_pub = rospy.Publisher(
            topics["diagnostics"], String, queue_size=1
        )

        self.state_lock = threading.Lock()
        self.dispatch_lock = threading.Lock()
        self.ego_buffer = deque()
        self.object_buffer = deque()
        self.pending_clouds = deque()
        self.cloud_sub = rospy.Subscriber(
            topics["point_cloud"],
            PointCloud2,
            self.cloud_callback,
            queue_size=5,
        )
        self.ego_sub = rospy.Subscriber(
            topics["ego"],
            EgoVehicleStatus,
            self.ego_callback,
            queue_size=100,
        )
        self.object_sub = rospy.Subscriber(
            topics["objects"],
            ObjectStatusList,
            self.object_callback,
            queue_size=100,
        )
        self.pending_timer = rospy.Timer(
            rospy.Duration(0.02), self.pending_timer_callback
        )

        rospy.loginfo(
            "LiDAR ground truth interpolation ready: "
            "cloud=%s ego=%s objects=%s wait=%.0fms",
            topics["point_cloud"],
            topics["ego"],
            topics["objects"],
            self.max_wait_seconds * 1000.0,
        )

    @staticmethod
    def append_in_stamp_order(buffer, sample):
        if not buffer or sample.stamp >= buffer[-1].stamp:
            buffer.append(sample)
            return
        for index in range(len(buffer) - 1, -1, -1):
            if buffer[index].stamp <= sample.stamp:
                buffer.insert(index + 1, sample)
                return
        buffer.appendleft(sample)

    def prune_buffer(self, buffer):
        if not buffer:
            return
        cutoff = buffer[-1].stamp - self.buffer_seconds
        while len(buffer) > 2 and buffer[1].stamp < cutoff:
            buffer.popleft()

    @staticmethod
    def find_bracket(buffer, target_stamp):
        if not buffer:
            return None
        previous = None
        for sample in buffer:
            if sample.stamp == target_stamp:
                return sample, sample
            if sample.stamp > target_stamp:
                if previous is None:
                    return None
                return previous, sample
            previous = sample
        return None

    @staticmethod
    def nearest_sample(buffer, target_stamp):
        if not buffer:
            return None
        return min(buffer, key=lambda sample: abs(sample.stamp - target_stamp))

    def ego_callback(self, message):
        sample = EgoState.from_message(message)
        with self.state_lock:
            self.append_in_stamp_order(self.ego_buffer, sample)
            self.prune_buffer(self.ego_buffer)
        self.dispatch_ready_clouds()

    def object_callback(self, message):
        sample = ObjectFrame.from_message(message)
        with self.state_lock:
            self.append_in_stamp_order(self.object_buffer, sample)
            self.prune_buffer(self.object_buffer)
        self.dispatch_ready_clouds()

    def cloud_callback(self, cloud):
        with self.state_lock:
            self.pending_clouds.append((cloud, time.monotonic()))
        self.dispatch_ready_clouds()

    def pending_timer_callback(self, _event):
        self.dispatch_ready_clouds()

    @staticmethod
    def bracket_diagnostics(bracket, target_stamp):
        return {
            "before_ms": round(
                1000.0 * (target_stamp - bracket[0].stamp), 3
            ),
            "after_ms": round(
                1000.0 * (bracket[1].stamp - target_stamp), 3
            ),
        }

    def select_state_locked(self, buffer, target_stamp, timed_out):
        bracket = self.find_bracket(buffer, target_stamp)
        if bracket is not None:
            return bracket, "interpolated"
        if not timed_out:
            return None, None
        nearest = self.nearest_sample(buffer, target_stamp)
        if (
            nearest is None
            or abs(nearest.stamp - target_stamp) > self.max_nearest_seconds
        ):
            return None, None
        return (nearest, nearest), "nearest_fallback"

    def dispatch_ready_clouds(self):
        if not self.dispatch_lock.acquire(False):
            return
        try:
            self._dispatch_ready_clouds()
        finally:
            self.dispatch_lock.release()

    def _dispatch_ready_clouds(self):
        work = []
        now = time.monotonic()
        with self.state_lock:
            while self.pending_clouds:
                cloud, arrival_time = self.pending_clouds[0]
                target_stamp = cloud.header.stamp.to_sec()
                timed_out = now - arrival_time >= self.max_wait_seconds
                ego_bracket, ego_mode = self.select_state_locked(
                    self.ego_buffer, target_stamp, timed_out
                )
                object_bracket, object_mode = self.select_state_locked(
                    self.object_buffer, target_stamp, timed_out
                )
                if ego_bracket is None or object_bracket is None:
                    if timed_out:
                        rospy.logwarn_throttle(
                            2.0,
                            "Dropping LiDAR frame: no state within %.0fms",
                            self.max_nearest_seconds * 1000.0,
                        )
                        self.pending_clouds.popleft()
                        continue
                    break
                self.pending_clouds.popleft()
                work.append(
                    (
                        cloud,
                        ego_bracket,
                        object_bracket,
                        ego_mode,
                        object_mode,
                        now - arrival_time,
                    )
                )

        for item in work:
            cloud = item[0]
            target_stamp = cloud.header.stamp.to_sec()
            ego = EgoState.interpolate(item[1][0], item[1][1], target_stamp)
            objects = ObjectFrame.interpolate(
                item[2][0], item[2][1], target_stamp
            )
            sync_diagnostics = {
                "mode": (
                    "interpolated"
                    if item[3] == "interpolated"
                    and item[4] == "interpolated"
                    else "nearest_fallback"
                ),
                "wait_ms": round(item[5] * 1000.0, 3),
                "ego": self.bracket_diagnostics(item[1], target_stamp),
                "objects": self.bracket_diagnostics(item[2], target_stamp),
            }
            self.process_cloud(cloud, ego, objects, sync_diagnostics)

    def object_in_lidar_frame(self, ego, obj):
        ego_yaw = math.radians(float(ego.heading))
        cosine = math.cos(ego_yaw)
        sine = math.sin(ego_yaw)

        delta_x = float(obj.position.x) - float(ego.position.x)
        delta_y = float(obj.position.y) - float(ego.position.y)
        vehicle_x = cosine * delta_x + sine * delta_y
        vehicle_y = -sine * delta_x + cosine * delta_y
        vehicle_z = float(obj.position.z) - float(ego.position.z)

        offset_x = vehicle_x - self.lidar_x
        offset_y = vehicle_y - self.lidar_y
        lidar_cosine = math.cos(self.lidar_yaw)
        lidar_sine = math.sin(self.lidar_yaw)
        lidar_x = lidar_cosine * offset_x + lidar_sine * offset_y
        lidar_y = -lidar_sine * offset_x + lidar_cosine * offset_y

        lidar_z = vehicle_z - self.lidar_z
        if self.position_is_bottom:
            lidar_z += 0.5 * float(obj.size.z)

        relative_yaw = normalize_angle(
            math.radians(float(obj.heading)) - ego_yaw - self.lidar_yaw
        )
        return lidar_x, lidar_y, lidar_z, relative_yaw

    def point_count(self, points, center, size, yaw):
        if points is None or points.size == 0:
            return 0
        delta_x = points[:, 0] - center[0]
        delta_y = points[:, 1] - center[1]
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        box_x = cosine * delta_x + sine * delta_y
        box_y = -sine * delta_x + cosine * delta_y
        inside = (
            (np.abs(box_x) <= 0.5 * size[0] + self.padding_xy)
            & (np.abs(box_y) <= 0.5 * size[1] + self.padding_xy)
            & (np.abs(points[:, 2] - center[2])
               <= 0.5 * size[2] + self.padding_z)
        )
        return int(np.count_nonzero(inside))

    @staticmethod
    def make_header(marker, cloud):
        marker.header.stamp = cloud.header.stamp
        marker.header.frame_id = cloud.header.frame_id

    def cube_marker(self, cloud, marker_id, category, obj, center, yaw):
        marker = Marker()
        self.make_header(marker, cloud)
        marker.ns = "ground_truth_boxes"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        marker.scale.x = max(0.01, float(obj.size.x))
        marker.scale.y = max(0.01, float(obj.size.y))
        marker.scale.z = max(0.01, float(obj.size.z))
        color = self.CATEGORY_COLORS[category]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = self.box_alpha
        marker.lifetime = rospy.Duration(self.marker_lifetime)
        return marker

    def outline_marker(self, cloud, marker_id, category, obj, center, yaw):
        marker = Marker()
        self.make_header(marker, cloud)
        marker.ns = "ground_truth_outlines"
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        color = self.CATEGORY_COLORS[category]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(self.marker_lifetime)

        half_x = 0.5 * float(obj.size.x)
        half_y = 0.5 * float(obj.size.y)
        half_z = 0.5 * float(obj.size.z)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        corners = []
        for z_sign in (-1.0, 1.0):
            for x_sign, y_sign in (
                (-1.0, -1.0),
                (1.0, -1.0),
                (1.0, 1.0),
                (-1.0, 1.0),
            ):
                local_x = x_sign * half_x
                local_y = y_sign * half_y
                point = type(marker.pose.position)()
                point.x = center[0] + cosine * local_x - sine * local_y
                point.y = center[1] + sine * local_x + cosine * local_y
                point.z = center[2] + z_sign * half_z
                corners.append(point)
        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )
        for first, second in edges:
            marker.points.append(corners[first])
            marker.points.append(corners[second])
        return marker

    def text_marker(
        self, cloud, marker_id, category, obj, center, point_count
    ):
        marker = Marker()
        self.make_header(marker, cloud)
        marker.ns = "ground_truth_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2] + 0.5 * float(obj.size.z) + 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.55
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = "{}:{} {} pts={}".format(
            category, obj.unique_id, obj.name, point_count
        )
        marker.lifetime = rospy.Duration(self.marker_lifetime)
        return marker

    def process_cloud(self, cloud, ego, objects, sync_diagnostics):
        points = None
        if self.count_points:
            tuples = list(
                point_cloud2.read_points(
                    cloud, field_names=("x", "y", "z"), skip_nans=True
                )
            )
            points = np.asarray(tuples, dtype=np.float32)

        marker_array = MarkerArray()
        delete = Marker()
        self.make_header(delete, cloud)
        delete.action = Marker.DELETEALL
        marker_array.markers.append(delete)

        diagnostics = []
        ordered_objects = sorted(
            objects.objects.values(),
            key=lambda item: (item.category, item.unique_id),
        )
        for index, obj in enumerate(ordered_objects):
            category = obj.category
            center_x, center_y, center_z, yaw = self.object_in_lidar_frame(
                ego, obj
            )
            center = (center_x, center_y, center_z)
            size = (
                float(obj.size.x),
                float(obj.size.y),
                float(obj.size.z),
            )
            count = self.point_count(points, center, size, yaw)
            marker_array.markers.append(
                self.cube_marker(cloud, index, category, obj, center, yaw)
            )
            marker_array.markers.append(
                self.outline_marker(cloud, index, category, obj, center, yaw)
            )
            if self.show_text:
                marker_array.markers.append(
                    self.text_marker(
                        cloud, index, category, obj, center, count
                    )
                )
            diagnostics.append(
                {
                    "category": category,
                    "id": int(obj.unique_id),
                    "name": obj.name,
                    "center": [center_x, center_y, center_z],
                    "size": list(size),
                    "yaw_rad": yaw,
                    "yaw_deg": math.degrees(yaw),
                    "points_in_box": count,
                }
            )

        stamp_ns = (
            int(cloud.header.stamp.secs) * 1000000000
            + int(cloud.header.stamp.nsecs)
        )
        self.marker_pub.publish(marker_array)
        self.diagnostic_pub.publish(
            String(
                data=json.dumps(
                    {
                        "stamp": cloud.header.stamp.to_sec(),
                        "stamp_ns": stamp_ns,
                        "frame_id": cloud.header.frame_id,
                        "ego": {
                            "position": [
                                float(ego.position.x),
                                float(ego.position.y),
                                float(ego.position.z),
                            ],
                            "heading_deg": float(ego.heading),
                        },
                        "sync": sync_diagnostics,
                        "objects": diagnostics,
                    },
                    ensure_ascii=False,
                    separators=(",", ":"),
                )
            )
        )
        rospy.loginfo_throttle(
            2.0,
            "Ground truth boxes=%d points=%d",
            len(diagnostics),
            0 if points is None else len(points),
        )


def main():
    rospy.init_node("lidar_ground_truth_visualizer")
    GroundTruthVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
