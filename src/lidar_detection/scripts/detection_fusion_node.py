#!/usr/bin/env python3
"""Fuse independent Car and Pedestrian PointPillars outputs for RViz.

The two archived models are single-class models.  A vehicle can therefore
occasionally produce a Pedestrian candidate as well as a Car candidate.  This
node synchronizes their JSON outputs by LiDAR stamp and removes pedestrian
centres that fall inside a detected vehicle box before publishing one marker
array and one combined status topic.
"""

import json
import math
import threading

import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def stamp_key(status):
    stamp = status["stamp"]
    return int(stamp["secs"]), int(stamp["nsecs"])


def pedestrian_inside_vehicle(pedestrian, vehicle, padding):
    """Return True when a pedestrian centre lies in a rotated vehicle box."""
    px, py, pz = (float(value) for value in pedestrian["center"])
    vx, vy, vz = (float(value) for value in vehicle["center"])
    length, width, height = (float(value) for value in vehicle["size"])
    yaw = float(vehicle["yaw_rad"])

    delta_x = px - vx
    delta_y = py - vy
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    box_x = cosine * delta_x + sine * delta_y
    box_y = -sine * delta_x + cosine * delta_y

    return (
        abs(box_x) <= 0.5 * length + padding
        and abs(box_y) <= 0.5 * width + padding
        and abs(pz - vz) <= 0.5 * height + padding
    )


def suppress_vehicle_overlaps(vehicles, pedestrians, padding):
    kept = []
    suppressed = []
    for pedestrian in pedestrians:
        if any(
            pedestrian_inside_vehicle(pedestrian, vehicle, padding)
            for vehicle in vehicles
        ):
            suppressed.append(pedestrian)
        else:
            kept.append(pedestrian)
    return kept, suppressed


class DetectionFusionNode:
    def __init__(self):
        self.car_topic = rospy.get_param(
            "~car_detections_topic", "/lidar_detection/detections"
        )
        self.pedestrian_topic = rospy.get_param(
            "~pedestrian_detections_topic",
            "/lidar_detection/pedestrian_far_v2_detections",
        )
        self.markers_topic = rospy.get_param(
            "~markers_topic", "/lidar_detection/combined_markers"
        )
        self.detections_topic = rospy.get_param(
            "~detections_topic", "/lidar_detection/combined_detections"
        )
        self.vehicle_box_padding = float(
            rospy.get_param("~vehicle_box_padding", 0.20)
        )
        self.marker_lifetime = float(rospy.get_param("~marker_lifetime", 0.30))
        self.max_pending_frames = int(rospy.get_param("~max_pending_frames", 40))
        if self.vehicle_box_padding < 0.0:
            raise ValueError("vehicle_box_padding must be non-negative")
        if self.max_pending_frames < 2:
            raise ValueError("max_pending_frames must be at least 2")

        self.pending = {"car": {}, "pedestrian": {}}
        self.pending_lock = threading.Lock()
        self.marker_pub = rospy.Publisher(
            self.markers_topic, MarkerArray, queue_size=1
        )
        self.detection_pub = rospy.Publisher(
            self.detections_topic, String, queue_size=1
        )
        self.car_sub = rospy.Subscriber(
            self.car_topic, String, self._car_callback, queue_size=20
        )
        self.pedestrian_sub = rospy.Subscriber(
            self.pedestrian_topic,
            String,
            self._pedestrian_callback,
            queue_size=20,
        )
        rospy.loginfo(
            "Detection fusion ready: car=%s pedestrian=%s output=%s padding=%.2fm",
            self.car_topic,
            self.pedestrian_topic,
            self.markers_topic,
            self.vehicle_box_padding,
        )

    def _car_callback(self, message):
        self._receive("car", message)

    def _pedestrian_callback(self, message):
        self._receive("pedestrian", message)

    def _receive(self, kind, message):
        try:
            status = json.loads(message.data)
            key = stamp_key(status)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            rospy.logerr_throttle(2.0, "Invalid %s detection JSON: %s", kind, error)
            return

        matched = None
        with self.pending_lock:
            self.pending[kind][key] = status
            other = "pedestrian" if kind == "car" else "car"
            if key in self.pending[other]:
                car_status = (
                    status if kind == "car" else self.pending["car"].pop(key)
                )
                pedestrian_status = (
                    status
                    if kind == "pedestrian"
                    else self.pending["pedestrian"].pop(key)
                )
                self.pending[kind].pop(key, None)
                matched = (car_status, pedestrian_status)
            self._prune()

        if matched is not None:
            self._publish(key, matched[0], matched[1])

    def _prune(self):
        for frames in self.pending.values():
            overflow = len(frames) - self.max_pending_frames
            if overflow > 0:
                for key in sorted(frames)[:overflow]:
                    frames.pop(key, None)

    def _append_box(self, marker_array, detection, namespace, marker_id, color):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.stamp
        marker.ns = namespace + "_boxes"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(detection["center"][0])
        marker.pose.position.y = float(detection["center"][1])
        marker.pose.position.z = float(detection["center"][2])
        half_yaw = float(detection["yaw_rad"]) * 0.5
        marker.pose.orientation.z = math.sin(half_yaw)
        marker.pose.orientation.w = math.cos(half_yaw)
        marker.scale.x = max(float(detection["size"][0]), 0.01)
        marker.scale.y = max(float(detection["size"][1]), 0.01)
        marker.scale.z = max(float(detection["size"][2]), 0.01)
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 0.30
        marker.lifetime = rospy.Duration.from_sec(self.marker_lifetime)
        marker_array.markers.append(marker)

        label = Marker()
        label.header = marker.header
        label.ns = namespace + "_labels"
        label.id = marker_id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = marker.pose.position.x
        label.pose.position.y = marker.pose.position.y
        label.pose.position.z = marker.pose.position.z + 0.5 * marker.scale.z + 0.5
        label.pose.orientation.w = 1.0
        label.scale.z = 0.55
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = "{} {:.2f}".format(
            detection["class_name"], float(detection["score"])
        )
        label.lifetime = marker.lifetime
        marker_array.markers.append(label)

    def _publish(self, key, car_status, pedestrian_status):
        vehicles = list(car_status.get("detections", []))
        raw_pedestrians = list(pedestrian_status.get("detections", []))
        pedestrians, suppressed = suppress_vehicle_overlaps(
            vehicles, raw_pedestrians, self.vehicle_box_padding
        )

        self.frame_id = str(
            pedestrian_status.get("frame_id")
            or car_status.get("frame_id")
            or "velodyne"
        )
        self.stamp = rospy.Time(secs=key[0], nsecs=key[1])
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.header.stamp = self.stamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for index, vehicle in enumerate(vehicles):
            self._append_box(markers, vehicle, "combined_car", index, (0.1, 0.45, 1.0))
        for index, pedestrian in enumerate(pedestrians):
            self._append_box(
                markers,
                pedestrian,
                "combined_pedestrian",
                index,
                (0.1, 1.0, 0.25),
            )
        self.marker_pub.publish(markers)

        combined = {
            "stamp": {"secs": key[0], "nsecs": key[1]},
            "frame_id": self.frame_id,
            "num_cars": len(vehicles),
            "num_pedestrians": len(pedestrians),
            "num_suppressed_pedestrians": len(suppressed),
            "detections": vehicles + pedestrians,
            "suppressed_pedestrians": suppressed,
            "car_latency_ms": car_status.get("latency_ms"),
            "pedestrian_latency_ms": pedestrian_status.get("latency_ms"),
        }
        self.detection_pub.publish(
            String(data=json.dumps(combined, ensure_ascii=False, separators=(",", ":")))
        )
        rospy.loginfo_throttle(
            2.0,
            "Fused detections cars=%d pedestrians=%d suppressed=%d",
            len(vehicles),
            len(pedestrians),
            len(suppressed),
        )


def main():
    rospy.init_node("car_pedestrian_detection_fusion")
    DetectionFusionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
