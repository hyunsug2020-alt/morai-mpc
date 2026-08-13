#!/usr/bin/env python3
"""Run the trained MORAI PointPillars model on a ROS PointCloud2 stream."""

import json
import logging
import math
import os
import sys
import threading
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
OPENPCDET_ROOT = PROJECT_ROOT / "third_party" / "OpenPCDet"
DEFAULT_VENV_PYTHON = PROJECT_ROOT / ".venv-openpcdet" / "bin" / "python"


def _ensure_runtime_python():
    """Use the OpenPCDet virtualenv even when started through rosrun."""
    requested = Path(
        os.environ.get("MORAI_POINTPILLAR_PYTHON", str(DEFAULT_VENV_PYTHON))
    )
    try:
        same_python = requested.exists() and Path(sys.executable).samefile(requested)
    except OSError:
        same_python = False
    if requested.exists() and not same_python:
        os.execv(str(requested), [str(requested), str(Path(__file__).resolve()), *sys.argv[1:]])


_ensure_runtime_python()

# ROS Noetic's Python modules are installed outside a normally isolated venv.
for _path in (
    "/usr/lib/python3/dist-packages",
    "/opt/ros/noetic/lib/python3/dist-packages",
    str(OPENPCDET_ROOT),
):
    if _path not in sys.path:
        sys.path.append(_path)

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import torch
import yaml
from easydict import EasyDict
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from pcdet.config import merge_new_config
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu


FILTERED_POINT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def _merge_dict(base, override):
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge_dict(result[key], value)
        else:
            result[key] = value
    return result


def load_runtime_config(model_config_path, data_config_path):
    """Load the archived model/data YAML without depending on the process cwd."""
    with open(data_config_path, "r", encoding="utf-8") as stream:
        data_document = yaml.safe_load(stream)
    with open(model_config_path, "r", encoding="utf-8") as stream:
        model_document = yaml.safe_load(stream)

    model_data = dict(model_document.get("DATA_CONFIG", {}))
    model_data.pop("_BASE_CONFIG_", None)
    model_document["DATA_CONFIG"] = _merge_dict(data_document, model_data)

    config = EasyDict()
    config.ROOT_DIR = OPENPCDET_ROOT
    config.LOCAL_RANK = 0
    merge_new_config(config, model_document)
    return config


class LivePointCloudDataset(DatasetTemplate):
    """Minimal OpenPCDet dataset adapter for points already held in memory."""

    def __init__(self, dataset_cfg, class_names, logger):
        super().__init__(
            dataset_cfg=dataset_cfg,
            class_names=class_names,
            training=False,
            root_path=PROJECT_ROOT / "datasets" / "morai_openpcdet",
            logger=logger,
        )

    def __len__(self):
        return 0

    def __getitem__(self, _index):
        raise IndexError("live dataset has no indexed samples")

    def prepare_cloud(self, points, frame_id):
        return self.prepare_data(
            data_dict={"points": points, "frame_id": str(frame_id)}
        )


class PointPillarRuntime:
    """Own the CUDA model and expose synchronous NumPy inference."""

    def __init__(self, model_config, data_config, checkpoint):
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA GPU is required but torch.cuda.is_available() is false")

        self.logger = logging.getLogger("morai_pointpillar")
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(logging.Formatter("[PointPillars] %(message)s"))
            self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)

        self.config = load_runtime_config(model_config, data_config)
        self.dataset = LivePointCloudDataset(
            self.config.DATA_CONFIG, self.config.CLASS_NAMES, self.logger
        )
        self.model = build_network(
            model_cfg=self.config.MODEL,
            num_class=len(self.config.CLASS_NAMES),
            dataset=self.dataset,
        )
        self.model.load_params_from_file(
            filename=str(checkpoint), logger=self.logger, to_cpu=True
        )
        self.model.cuda()
        self.model.eval()

    def infer(self, points, frame_id):
        prepared = self.dataset.prepare_cloud(points, frame_id)
        batch = self.dataset.collate_batch([prepared])
        load_data_to_gpu(batch)
        with torch.inference_mode():
            predictions, _ = self.model.forward(batch)
        prediction = predictions[0]
        return (
            prediction["pred_boxes"].detach().cpu().numpy(),
            prediction["pred_scores"].detach().cpu().numpy(),
            prediction["pred_labels"].detach().cpu().numpy(),
        )


def pointcloud2_to_xyzi(cloud):
    """Match the x/y/z/intensity conversion used to create the training set."""
    field_names = {field.name for field in cloud.fields}
    missing = {"x", "y", "z"} - field_names
    if missing:
        raise ValueError("PointCloud2 is missing fields: {}".format(sorted(missing)))

    if "intensity" in field_names:
        rows = point_cloud2.read_points(
            cloud,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True,
        )
        points = np.asarray(list(rows), dtype=np.float32)
    else:
        rows = point_cloud2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=True
        )
        xyz = np.asarray(list(rows), dtype=np.float32).reshape((-1, 3))
        points = np.zeros((len(xyz), 4), dtype=np.float32)
        points[:, :3] = xyz

    return points.reshape((-1, 4))


def filter_ground_ransac(
    points,
    random_generator,
    distance_threshold,
    iterations,
    max_tilt_deg,
    min_range,
    max_range,
    candidate_max_z,
    min_plane_height,
    max_plane_height,
    sample_size,
    min_inliers,
):
    """Remove only the dominant near-horizontal road plane from XYZI points."""
    if len(points) < 3:
        return points, 0, None

    xyz = points[:, :3]
    finite = np.isfinite(xyz).all(axis=1)
    ranges_squared = np.square(xyz[:, 0]) + np.square(xyz[:, 1])
    candidates = xyz[
        finite
        & (ranges_squared >= min_range * min_range)
        & (ranges_squared <= max_range * max_range)
        & (xyz[:, 2] <= candidate_max_z)
    ]
    if len(candidates) < min_inliers:
        return points, 0, None

    if len(candidates) > sample_size:
        fit_points = candidates[
            random_generator.choice(len(candidates), sample_size, replace=False)
        ]
    else:
        fit_points = candidates

    minimum_normal_z = math.cos(math.radians(max_tilt_deg))
    best_count = 0
    best_normal = None
    best_offset = None
    for _ in range(iterations):
        sample = fit_points[
            random_generator.choice(len(fit_points), size=3, replace=False)
        ]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        normal_length = float(np.linalg.norm(normal))
        if normal_length < 1e-6:
            continue
        normal /= normal_length
        if normal[2] < 0.0:
            normal = -normal
        if normal[2] < minimum_normal_z:
            continue

        offset = -float(np.dot(normal, sample[0]))
        plane_height = -offset / float(normal[2])
        if not min_plane_height <= plane_height <= max_plane_height:
            continue

        count = int(
            np.count_nonzero(
                np.abs(np.matmul(fit_points, normal) + offset)
                <= distance_threshold
            )
        )
        if count > best_count:
            best_count = count
            best_normal = normal
            best_offset = offset

    if best_normal is None:
        return points, 0, None

    candidate_distances = np.abs(
        np.matmul(candidates, best_normal) + best_offset
    )
    plane_points = candidates[candidate_distances <= distance_threshold]
    if len(plane_points) < min_inliers:
        return points, 0, None

    # Refine the sampled plane with all of its candidate inliers.  The
    # smallest covariance eigenvector is the least-squares plane normal.
    plane_points = plane_points.astype(np.float64, copy=False)
    centroid = plane_points.mean(axis=0)
    centered = plane_points - centroid
    covariance = np.matmul(centered.T, centered)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if eigenvalues[1] < 1e-9:
        return points, 0, None
    normal = eigenvectors[:, 0]
    if normal[2] < 0.0:
        normal = -normal
    if normal[2] < minimum_normal_z:
        return points, 0, None

    offset = -float(np.dot(normal, centroid))
    plane_height = -offset / float(normal[2])
    if not min_plane_height <= plane_height <= max_plane_height:
        return points, 0, None

    ground_mask = finite & (
        np.abs(np.matmul(xyz, normal) + offset) <= distance_threshold
    )
    removed_count = int(np.count_nonzero(ground_mask))
    if removed_count < min_inliers:
        return points, 0, None

    plane = {
        "normal": [float(value) for value in normal],
        "offset": offset,
        "height_at_sensor": plane_height,
        "tilt_deg": math.degrees(
            math.acos(float(np.clip(normal[2], -1.0, 1.0)))
        ),
    }
    return points[~ground_mask], removed_count, plane


class PointPillarDetectorNode:
    def __init__(self):
        model_dir = PROJECT_ROOT / "trained_models" / "morai_pointpillar_2026-07-31"
        model_config = Path(
            rospy.get_param("~model_config", str(model_dir / "pointpillar_morai.yaml"))
        ).expanduser().resolve()
        data_config = Path(
            rospy.get_param("~data_config", str(model_dir / "morai_dataset.yaml"))
        ).expanduser().resolve()
        checkpoint = Path(
            rospy.get_param("~checkpoint", str(model_dir / "best_model.pth"))
        ).expanduser().resolve()

        for description, path in (
            ("model config", model_config),
            ("data config", data_config),
            ("checkpoint", checkpoint),
        ):
            if not path.is_file():
                raise FileNotFoundError("{} not found: {}".format(description, path))

        self.point_cloud_topic = rospy.get_param("~point_cloud_topic", "/velodyne_points")
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/lidar_detection/detection_markers"
        )
        self.detections_topic = rospy.get_param(
            "~detections_topic", "/lidar_detection/detections"
        )
        self.score_threshold = float(rospy.get_param("~score_threshold", 0.35))
        self.max_range = float(rospy.get_param("~max_range", 70.4))
        self.marker_lifetime = float(rospy.get_param("~marker_lifetime", 0.3))
        self.output_frame = str(rospy.get_param("~output_frame", ""))
        self.publish_labels = bool(rospy.get_param("~publish_labels", True))
        self.remove_ground = bool(rospy.get_param("~remove_ground", False))
        self.filtered_points_topic = str(
            rospy.get_param("~filtered_points_topic", "")
        )
        self.ground_distance_threshold = float(
            rospy.get_param("~ground_distance_threshold", 0.15)
        )
        self.ground_ransac_iterations = int(
            rospy.get_param("~ground_ransac_iterations", 48)
        )
        self.ground_max_tilt_deg = float(
            rospy.get_param("~ground_max_tilt_deg", 12.0)
        )
        self.ground_min_range = float(rospy.get_param("~ground_min_range", 2.0))
        self.ground_max_range = float(rospy.get_param("~ground_max_range", 50.0))
        self.ground_candidate_max_z = float(
            rospy.get_param("~ground_candidate_max_z", -0.5)
        )
        self.ground_min_plane_height = float(
            rospy.get_param("~ground_min_plane_height", -2.5)
        )
        self.ground_max_plane_height = float(
            rospy.get_param("~ground_max_plane_height", -0.7)
        )
        self.ground_ransac_sample_size = int(
            rospy.get_param("~ground_ransac_sample_size", 2048)
        )
        self.ground_min_inliers = int(
            rospy.get_param("~ground_min_inliers", 300)
        )
        self.ground_random = np.random.default_rng(20260811)
        self.model_epoch = int(rospy.get_param("~model_epoch", 39))
        self.checkpoint_label = str(
            rospy.get_param("~checkpoint_label", checkpoint.name)
        )
        class_filter_value = rospy.get_param("~class_filter", "")
        if isinstance(class_filter_value, list):
            requested_classes = [str(value).strip() for value in class_filter_value]
        else:
            requested_classes = [
                value.strip() for value in str(class_filter_value).split(",")
            ]
        self.requested_classes = {value for value in requested_classes if value}

        if not 0.0 <= self.score_threshold <= 1.0:
            raise ValueError("score_threshold must be in [0, 1]")
        if self.max_range <= 0.0:
            raise ValueError("max_range must be positive")
        if self.ground_distance_threshold <= 0.0:
            raise ValueError("ground_distance_threshold must be positive")
        if self.ground_ransac_iterations <= 0:
            raise ValueError("ground_ransac_iterations must be positive")
        if not 0.0 <= self.ground_max_tilt_deg < 90.0:
            raise ValueError("ground_max_tilt_deg must be in [0, 90)")
        if not 0.0 <= self.ground_min_range < self.ground_max_range:
            raise ValueError("ground range must satisfy 0 <= min < max")
        if self.ground_min_plane_height >= self.ground_max_plane_height:
            raise ValueError("ground plane height bounds are invalid")
        if self.ground_ransac_sample_size < 3:
            raise ValueError("ground_ransac_sample_size must be at least 3")
        if self.ground_min_inliers < 3:
            raise ValueError("ground_min_inliers must be at least 3")

        rospy.loginfo(
            "Loading epoch %d PointPillars checkpoint: %s",
            self.model_epoch,
            checkpoint,
        )
        self.runtime = PointPillarRuntime(model_config, data_config, checkpoint)
        self.class_names = list(self.runtime.config.CLASS_NAMES)
        unknown_classes = self.requested_classes - set(self.class_names)
        if unknown_classes:
            raise ValueError(
                "class_filter contains classes absent from model: {}".format(
                    sorted(unknown_classes)
                )
            )
        self.allowed_label_ids = {
            index + 1
            for index, class_name in enumerate(self.class_names)
            if not self.requested_classes or class_name in self.requested_classes
        }

        self.marker_pub = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1
        )
        self.detections_pub = rospy.Publisher(
            self.detections_topic, String, queue_size=1
        )
        self.filtered_points_pub = (
            rospy.Publisher(
                self.filtered_points_topic, PointCloud2, queue_size=1
            )
            if self.filtered_points_topic
            else None
        )

        self.condition = threading.Condition()
        self.latest_cloud = None
        self.stop_requested = False
        self.received_frames = 0
        self.processed_frames = 0
        self.dropped_frames = 0
        self.failed_frames = 0
        self.worker = threading.Thread(
            target=self._worker_loop, name="pointpillar-inference", daemon=True
        )
        self.worker.start()

        self.cloud_sub = rospy.Subscriber(
            self.point_cloud_topic,
            PointCloud2,
            self._cloud_callback,
            queue_size=1,
            buff_size=16 * 1024 * 1024,
        )
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "PointPillars ready: input=%s markers=%s detections=%s classes=%s score>=%.2f",
            self.point_cloud_topic,
            self.marker_topic,
            self.detections_topic,
            sorted(self.requested_classes) if self.requested_classes else self.class_names,
            self.score_threshold,
        )
        if self.remove_ground:
            rospy.loginfo(
                "Ground RANSAC enabled: distance=%.2fm iterations=%d tilt<=%.1fdeg "
                "height=[%.2f, %.2f]m filtered_topic=%s",
                self.ground_distance_threshold,
                self.ground_ransac_iterations,
                self.ground_max_tilt_deg,
                self.ground_min_plane_height,
                self.ground_max_plane_height,
                self.filtered_points_topic or "disabled",
            )

    def _cloud_callback(self, cloud):
        with self.condition:
            self.received_frames += 1
            if self.latest_cloud is not None:
                self.dropped_frames += 1
            self.latest_cloud = cloud
            self.condition.notify()

    def _worker_loop(self):
        while True:
            with self.condition:
                while self.latest_cloud is None and not self.stop_requested:
                    self.condition.wait(timeout=0.5)
                if self.stop_requested:
                    return
                cloud = self.latest_cloud
                self.latest_cloud = None
            self._process_cloud(cloud)

    def _process_cloud(self, cloud):
        started = time.perf_counter()
        try:
            points = pointcloud2_to_xyzi(cloud)
            raw_point_count = len(points)
            ground_removed_count = 0
            ground_plane = None
            if self.remove_ground and len(points):
                points, ground_removed_count, ground_plane = filter_ground_ransac(
                    points=points,
                    random_generator=self.ground_random,
                    distance_threshold=self.ground_distance_threshold,
                    iterations=self.ground_ransac_iterations,
                    max_tilt_deg=self.ground_max_tilt_deg,
                    min_range=self.ground_min_range,
                    max_range=self.ground_max_range,
                    candidate_max_z=self.ground_candidate_max_z,
                    min_plane_height=self.ground_min_plane_height,
                    max_plane_height=self.ground_max_plane_height,
                    sample_size=self.ground_ransac_sample_size,
                    min_inliers=self.ground_min_inliers,
                )

            self._publish_filtered_cloud(cloud, points)
            if not len(points):
                latency_ms = (time.perf_counter() - started) * 1000.0
                self._publish(
                    cloud,
                    points,
                    np.empty((0, 7)),
                    np.empty(0),
                    np.empty(0),
                    latency_ms,
                    raw_point_count,
                    ground_removed_count,
                    ground_plane,
                )
                self.processed_frames += 1
                return

            boxes, scores, labels = self.runtime.infer(
                points, "{}.{}".format(cloud.header.stamp.secs, cloud.header.stamp.nsecs)
            )
            ranges = np.hypot(boxes[:, 0], boxes[:, 1]) if len(boxes) else np.empty(0)
            allowed = np.isin(labels, list(self.allowed_label_ids))
            selected = (
                (scores >= self.score_threshold)
                & (ranges <= self.max_range)
                & allowed
            )
            boxes = boxes[selected]
            scores = scores[selected]
            labels = labels[selected]
            latency_ms = (time.perf_counter() - started) * 1000.0
            self._publish(
                cloud,
                points,
                boxes,
                scores,
                labels,
                latency_ms,
                raw_point_count,
                ground_removed_count,
                ground_plane,
            )
            self.processed_frames += 1
            rospy.loginfo_throttle(
                2.0,
                "PointPillars frames=%d dropped=%d raw=%d points=%d ground=%d "
                "detections=%d latency=%.1fms",
                self.processed_frames,
                self.dropped_frames,
                raw_point_count,
                len(points),
                ground_removed_count,
                len(boxes),
                latency_ms,
            )
        except Exception as error:
            self.failed_frames += 1
            rospy.logerr_throttle(2.0, "PointPillars inference failed: %s", error)
            if isinstance(error, torch.cuda.OutOfMemoryError):
                torch.cuda.empty_cache()

    def _publish_filtered_cloud(self, cloud, points):
        if self.filtered_points_pub is None:
            return
        packed_points = np.ascontiguousarray(points, dtype="<f4")
        filtered_cloud = PointCloud2()
        filtered_cloud.header = cloud.header
        filtered_cloud.height = 1
        filtered_cloud.width = len(packed_points)
        filtered_cloud.fields = FILTERED_POINT_FIELDS
        filtered_cloud.is_bigendian = False
        filtered_cloud.point_step = 16
        filtered_cloud.row_step = filtered_cloud.point_step * filtered_cloud.width
        filtered_cloud.data = packed_points.tobytes()
        filtered_cloud.is_dense = True
        self.filtered_points_pub.publish(filtered_cloud)

    def _publish(
        self,
        cloud,
        points,
        boxes,
        scores,
        labels,
        latency_ms,
        raw_point_count,
        ground_removed_count,
        ground_plane,
    ):
        frame_id = self.output_frame or cloud.header.frame_id or "velodyne"
        marker_array = MarkerArray()

        clear = Marker()
        clear.header.stamp = cloud.header.stamp
        clear.header.frame_id = frame_id
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        detections = []
        for index, (box, score, label) in enumerate(zip(boxes, scores, labels)):
            class_index = int(label) - 1
            class_name = (
                self.class_names[class_index]
                if 0 <= class_index < len(self.class_names)
                else "class_{}".format(int(label))
            )
            cube = Marker()
            cube.header.stamp = cloud.header.stamp
            cube.header.frame_id = frame_id
            cube.ns = "pointpillar_boxes"
            cube.id = index
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = float(box[0])
            cube.pose.position.y = float(box[1])
            cube.pose.position.z = float(box[2])
            half_yaw = float(box[6]) * 0.5
            cube.pose.orientation.z = math.sin(half_yaw)
            cube.pose.orientation.w = math.cos(half_yaw)
            cube.scale.x = max(float(box[3]), 0.01)
            cube.scale.y = max(float(box[4]), 0.01)
            cube.scale.z = max(float(box[5]), 0.01)
            cube.color.r = float(max(0.0, 1.0 - score))
            cube.color.g = float(min(1.0, score + 0.2))
            cube.color.b = 0.1
            cube.color.a = 0.30
            cube.lifetime = rospy.Duration.from_sec(self.marker_lifetime)
            marker_array.markers.append(cube)

            if self.publish_labels:
                text_marker = Marker()
                text_marker.header = cube.header
                text_marker.ns = "pointpillar_labels"
                text_marker.id = index
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = float(box[0])
                text_marker.pose.position.y = float(box[1])
                text_marker.pose.position.z = float(box[2] + box[5] * 0.5 + 0.5)
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.55
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                text_marker.text = "{} {:.2f}".format(class_name, float(score))
                text_marker.lifetime = cube.lifetime
                marker_array.markers.append(text_marker)

            detections.append(
                {
                    "class_name": class_name,
                    "class_id": int(label),
                    "score": round(float(score), 6),
                    "center": [round(float(value), 6) for value in box[:3]],
                    "size": [round(float(value), 6) for value in box[3:6]],
                    "yaw_rad": round(float(box[6]), 6),
                    "range_m": round(float(math.hypot(box[0], box[1])), 6),
                }
            )

        self.marker_pub.publish(marker_array)
        status = {
            "stamp": {"secs": int(cloud.header.stamp.secs), "nsecs": int(cloud.header.stamp.nsecs)},
            "frame_id": frame_id,
            "model_epoch": self.model_epoch,
            "checkpoint": self.checkpoint_label,
            "class_filter": sorted(self.requested_classes),
            "raw_num_points": int(raw_point_count),
            "num_points": int(len(points)),
            "ground_removed_points": int(ground_removed_count),
            "ground_plane": ground_plane,
            "num_detections": len(detections),
            "latency_ms": round(float(latency_ms), 3),
            "received_frames": self.received_frames,
            "processed_frames": self.processed_frames + 1,
            "dropped_frames": self.dropped_frames,
            "failed_frames": self.failed_frames,
            "detections": detections,
        }
        self.detections_pub.publish(
            String(data=json.dumps(status, ensure_ascii=False, separators=(",", ":")))
        )

    def shutdown(self):
        with self.condition:
            if self.stop_requested:
                return
            self.stop_requested = True
            self.latest_cloud = None
            self.condition.notify_all()
        self.worker.join(timeout=5.0)
        if self.worker.is_alive():
            rospy.logwarn("PointPillars worker did not stop within 5 seconds")


def main():
    rospy.init_node("pointpillar_detector")
    PointPillarDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
