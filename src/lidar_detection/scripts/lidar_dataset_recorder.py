#!/usr/bin/env python3
"""Record LiDAR points and interpolated MORAI ground-truth boxes."""

import json
import math
import os
import queue
import shutil
import threading
import time
from datetime import datetime

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse


NANOSECONDS_PER_SECOND = 1000000000


def cloud_stamp_ns(cloud):
    return (
        int(cloud.header.stamp.secs) * NANOSECONDS_PER_SECOND
        + int(cloud.header.stamp.nsecs)
    )


def diagnostic_stamp_ns(diagnostic):
    if "stamp_ns" in diagnostic:
        return int(diagnostic["stamp_ns"])
    return int(round(float(diagnostic["stamp"]) * NANOSECONDS_PER_SECOND))


class LidarDatasetRecorder:
    def __init__(self):
        topics = rospy.get_param("~topics")
        recording = rospy.get_param("~recording", {})
        filtering = rospy.get_param("~filtering", {})
        classes = rospy.get_param("~class_names", {"npc": "Car"})

        self.output_dir = os.path.abspath(
            os.path.expanduser(
                rospy.get_param("~output_dir", "datasets/morai_lidar")
            )
        )
        self.enabled = bool(rospy.get_param("~enabled", True))
        self.record_every_n = max(
            1, int(recording.get("record_every_n", 2))
        )
        self.empty_frame_every_n = max(
            1, int(recording.get("empty_frame_every_n", 10))
        )
        self.match_buffer_seconds = max(
            0.2, float(recording.get("match_buffer_seconds", 2.0))
        )
        self.writer_queue_size = max(
            2, int(recording.get("writer_queue_size", 50))
        )
        self.min_free_disk_gb = max(
            0.0, float(recording.get("min_free_disk_gb", 5.0))
        )
        self.require_interpolated = bool(
            recording.get("require_interpolated", True)
        )
        self.min_points_per_box = max(
            0, int(filtering.get("min_points_per_box", 5))
        )
        self.max_label_range_m = max(
            0.0, float(filtering.get("max_label_range_m", 70.0))
        )
        self.class_names = {
            str(category): str(class_name)
            for category, class_name in classes.items()
        }

        self.buffer_lock = threading.Lock()
        self.cloud_buffer = {}
        self.diagnostic_buffer = {}
        self.write_queue = queue.Queue(maxsize=self.writer_queue_size)
        self.stop_event = threading.Event()
        self.matched_frames = 0
        self.saved_frames = 0
        self.saved_objects = 0
        self.saved_empty_frames = 0
        self.vehicle_candidate_frames = 0
        self.empty_candidate_frames = 0
        self.skipped_frames = 0
        self.dropped_frames = 0
        self.writer_errors = 0
        self.last_saved_stamp_ns = None

        self.prepare_output()
        self.next_index = self.find_next_index()

        self.cloud_sub = rospy.Subscriber(
            topics["point_cloud"],
            PointCloud2,
            self.cloud_callback,
            queue_size=20,
        )
        self.diagnostic_sub = rospy.Subscriber(
            topics["diagnostics"],
            String,
            self.diagnostic_callback,
            queue_size=50,
        )
        self.enabled_service = rospy.Service(
            "~set_enabled", SetBool, self.set_enabled
        )
        self.status_pub = rospy.Publisher(
            topics["status"], String, queue_size=1, latch=True
        )
        self.status_timer = rospy.Timer(
            rospy.Duration(1.0), self.status_timer_callback
        )
        self.worker = threading.Thread(
            target=self.writer_loop, name="lidar-dataset-writer"
        )
        self.worker.daemon = True
        self.worker.start()
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "LiDAR dataset recorder ready: output=%s enabled=%s every_n=%d",
            self.output_dir,
            self.enabled,
            self.record_every_n,
        )

    @property
    def directories(self):
        return {
            "velodyne": os.path.join(self.output_dir, "velodyne"),
            "labels": os.path.join(self.output_dir, "labels"),
            "label_lidar": os.path.join(self.output_dir, "label_lidar"),
            "timestamps": os.path.join(self.output_dir, "timestamps"),
            "calib": os.path.join(self.output_dir, "calib"),
            "image_sets": os.path.join(self.output_dir, "ImageSets"),
        }

    def prepare_output(self):
        os.makedirs(self.output_dir, exist_ok=True)
        for path in self.directories.values():
            os.makedirs(path, exist_ok=True)

        format_description = {
            "format_version": 1,
            "coordinate_system": {
                "frame": "velodyne",
                "x": "forward",
                "y": "left",
                "z": "up",
                "yaw": "counter-clockwise around +z, radians",
            },
            "point_format": ["x", "y", "z", "intensity"],
            "point_dtype": "float32",
            "label_lidar_columns": [
                "class",
                "center_x",
                "center_y",
                "center_z",
                "size_x",
                "size_y",
                "size_z",
                "yaw_rad",
                "unique_id",
                "points_in_box",
            ],
            "class_names": self.class_names,
            "created_at": datetime.now().astimezone().isoformat(),
        }
        self.write_json_if_missing(
            os.path.join(self.output_dir, "dataset_format.json"),
            format_description,
        )
        self.write_json_if_missing(
            os.path.join(
                self.directories["calib"], "lidar_extrinsic.json"
            ),
            {
                "parent_frame": "vehicle",
                "child_frame": "velodyne",
                "translation_m": [1.045, 0.0, 1.234],
                "rotation_rpy_deg": [0.0, 0.0, 0.0],
            },
        )

    @staticmethod
    def write_json_if_missing(path, value):
        if os.path.exists(path):
            return
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(
                value, stream, ensure_ascii=False, indent=2, sort_keys=True
            )
            stream.write("\n")
        os.replace(temporary, path)

    def find_next_index(self):
        highest = -1
        for name in os.listdir(self.directories["velodyne"]):
            stem, extension = os.path.splitext(name)
            if extension == ".bin" and stem.isdigit():
                highest = max(highest, int(stem))
        return highest + 1

    def set_enabled(self, request):
        self.enabled = bool(request.data)
        state = "enabled" if self.enabled else "paused"
        rospy.loginfo("LiDAR dataset recording %s", state)
        return SetBoolResponse(
            success=True,
            message="recording {}".format(state),
        )

    def cloud_callback(self, cloud):
        key = cloud_stamp_ns(cloud)
        with self.buffer_lock:
            self.cloud_buffer[key] = (cloud, time.monotonic())
            pair = self.pop_pair_locked(key)
            self.prune_buffers_locked()
        self.enqueue_pair(pair)

    def diagnostic_callback(self, message):
        try:
            diagnostic = json.loads(message.data)
            key = diagnostic_stamp_ns(diagnostic)
        except (KeyError, TypeError, ValueError) as error:
            rospy.logwarn_throttle(
                2.0, "Invalid ground-truth diagnostic: %s", error
            )
            return
        with self.buffer_lock:
            self.diagnostic_buffer[key] = (diagnostic, time.monotonic())
            pair = self.pop_pair_locked(key)
            self.prune_buffers_locked()
        self.enqueue_pair(pair)

    def pop_pair_locked(self, key):
        cloud_entry = self.cloud_buffer.get(key)
        diagnostic_entry = self.diagnostic_buffer.get(key)
        if cloud_entry is None or diagnostic_entry is None:
            return None
        del self.cloud_buffer[key]
        del self.diagnostic_buffer[key]
        return cloud_entry[0], diagnostic_entry[0]

    def prune_buffers_locked(self):
        cutoff = time.monotonic() - self.match_buffer_seconds
        for buffer in (self.cloud_buffer, self.diagnostic_buffer):
            expired = [
                key
                for key, (_value, arrival_time) in buffer.items()
                if arrival_time < cutoff
            ]
            for key in expired:
                del buffer[key]
                self.dropped_frames += 1

    def enqueue_pair(self, pair):
        if pair is None:
            return
        self.matched_frames += 1
        diagnostic = pair[1]
        sync_mode = diagnostic.get("sync", {}).get("mode", "unknown")
        if self.require_interpolated and sync_mode != "interpolated":
            self.skipped_frames += 1
            return
        if not self.enabled:
            self.skipped_frames += 1
            return
        if (self.matched_frames - 1) % self.record_every_n != 0:
            self.skipped_frames += 1
            return
        objects = self.selected_objects(diagnostic)
        if objects:
            self.vehicle_candidate_frames += 1
        else:
            self.empty_candidate_frames += 1
            if (
                (self.empty_candidate_frames - 1)
                % self.empty_frame_every_n
                != 0
            ):
                self.skipped_frames += 1
                return
        try:
            self.write_queue.put_nowait((pair[0], pair[1], objects))
        except queue.Full:
            self.dropped_frames += 1
            rospy.logwarn_throttle(
                2.0, "Dataset writer queue full; dropping frame"
            )

    def writer_loop(self):
        while not self.stop_event.is_set() or not self.write_queue.empty():
            try:
                item = self.write_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self.write_frame(item[0], item[1], item[2])
            except Exception as error:
                self.writer_errors += 1
                rospy.logerr("Failed to write LiDAR dataset frame: %s", error)
            finally:
                self.write_queue.task_done()

    @staticmethod
    def cloud_to_numpy(cloud):
        field_names = {field.name for field in cloud.fields}
        if "intensity" in field_names:
            rows = point_cloud2.read_points(
                cloud,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True,
            )
            points = np.asarray(list(rows), dtype=np.float32)
        else:
            rows = point_cloud2.read_points(
                cloud,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
            xyz = np.asarray(list(rows), dtype=np.float32)
            points = np.zeros((len(xyz), 4), dtype=np.float32)
            if len(xyz):
                points[:, :3] = xyz
        return points.reshape((-1, 4))

    def selected_objects(self, diagnostic):
        selected = []
        for obj in diagnostic.get("objects", []):
            category = str(obj.get("category", ""))
            if category not in self.class_names:
                continue
            if int(obj.get("points_in_box", 0)) < self.min_points_per_box:
                continue
            center = [float(value) for value in obj["center"]]
            if (
                self.max_label_range_m > 0.0
                and math.hypot(center[0], center[1])
                > self.max_label_range_m
            ):
                continue
            copied = dict(obj)
            copied["class_name"] = self.class_names[category]
            copied["center"] = center
            copied["size"] = [float(value) for value in obj["size"]]
            copied["yaw_rad"] = float(
                obj.get(
                    "yaw_rad",
                    math.radians(float(obj.get("yaw_deg", 0.0))),
                )
            )
            selected.append(copied)
        return selected

    def check_disk_space(self):
        free_gb = shutil.disk_usage(self.output_dir).free / (1024.0 ** 3)
        if free_gb < self.min_free_disk_gb:
            self.enabled = False
            raise RuntimeError(
                "free disk {:.2f}GB is below limit {:.2f}GB; recording paused"
                .format(free_gb, self.min_free_disk_gb)
            )

    def write_frame(self, cloud, diagnostic, objects):
        if self.saved_frames % 100 == 0:
            self.check_disk_space()

        points = self.cloud_to_numpy(cloud)
        frame_id = "{:06d}".format(self.next_index)
        stamp_ns = cloud_stamp_ns(cloud)
        seconds = int(cloud.header.stamp.secs)
        nanoseconds = int(cloud.header.stamp.nsecs)

        point_path = os.path.join(
            self.directories["velodyne"], frame_id + ".bin"
        )
        temporary_point_path = point_path + ".tmp"
        with open(temporary_point_path, "wb") as stream:
            points.astype(np.float32, copy=False).tofile(stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_point_path, point_path)

        label_lines = []
        for obj in objects:
            values = [
                obj["class_name"],
                *["{:.9f}".format(value) for value in obj["center"]],
                *["{:.9f}".format(value) for value in obj["size"]],
                "{:.9f}".format(obj["yaw_rad"]),
                str(int(obj["id"])),
                str(int(obj["points_in_box"])),
            ]
            label_lines.append(" ".join(values))
        self.atomic_write_text(
            os.path.join(
                self.directories["label_lidar"], frame_id + ".txt"
            ),
            "\n".join(label_lines) + ("\n" if label_lines else ""),
        )

        label_document = {
            "frame_id": frame_id,
            "stamp_ns": stamp_ns,
            "stamp": {
                "secs": seconds,
                "nsecs": nanoseconds,
            },
            "cloud_frame": cloud.header.frame_id,
            "num_points": int(len(points)),
            "ego": diagnostic.get("ego"),
            "sync": diagnostic.get("sync", {}),
            "objects": objects,
        }
        self.atomic_write_text(
            os.path.join(self.directories["labels"], frame_id + ".json"),
            json.dumps(
                label_document,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
            + "\n",
        )
        self.atomic_write_text(
            os.path.join(
                self.directories["timestamps"], frame_id + ".txt"
            ),
            "{}.{:09d}\n".format(seconds, nanoseconds),
        )

        with open(
            os.path.join(self.directories["image_sets"], "all.txt"),
            "a",
            encoding="utf-8",
        ) as stream:
            stream.write(frame_id + "\n")
            stream.flush()

        manifest_entry = {
            "frame_id": frame_id,
            "stamp_ns": stamp_ns,
            "num_points": int(len(points)),
            "num_objects": len(objects),
        }
        with open(
            os.path.join(self.output_dir, "manifest.jsonl"),
            "a",
            encoding="utf-8",
        ) as stream:
            stream.write(
                json.dumps(
                    manifest_entry,
                    ensure_ascii=False,
                    separators=(",", ":"),
                )
                + "\n"
            )
            stream.flush()

        self.next_index += 1
        self.saved_frames += 1
        self.saved_objects += len(objects)
        if not objects:
            self.saved_empty_frames += 1
        self.last_saved_stamp_ns = stamp_ns
        rospy.loginfo_throttle(
            2.0,
            "Dataset saved=%d frame=%s points=%d objects=%d",
            self.saved_frames,
            frame_id,
            len(points),
            len(objects),
        )

    @staticmethod
    def atomic_write_text(path, content):
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)

    def status_timer_callback(self, _event):
        try:
            free_gb = shutil.disk_usage(self.output_dir).free / (1024.0 ** 3)
        except OSError:
            free_gb = -1.0
        status = {
            "enabled": self.enabled,
            "output_dir": self.output_dir,
            "matched_frames": self.matched_frames,
            "saved_frames": self.saved_frames,
            "saved_objects": self.saved_objects,
            "saved_empty_frames": self.saved_empty_frames,
            "vehicle_candidate_frames": self.vehicle_candidate_frames,
            "empty_candidate_frames": self.empty_candidate_frames,
            "skipped_frames": self.skipped_frames,
            "dropped_frames": self.dropped_frames,
            "writer_errors": self.writer_errors,
            "writer_queue": self.write_queue.qsize(),
            "next_index": self.next_index,
            "free_disk_gb": round(free_gb, 3),
            "last_saved_stamp_ns": self.last_saved_stamp_ns,
        }
        self.status_pub.publish(
            String(
                data=json.dumps(
                    status, ensure_ascii=False, separators=(",", ":")
                )
            )
        )

    def shutdown(self):
        if self.stop_event.is_set():
            return
        self.enabled = False
        self.stop_event.set()
        self.worker.join(timeout=5.0)
        if self.worker.is_alive():
            rospy.logwarn(
                "Dataset writer did not finish before shutdown timeout"
            )


def main():
    rospy.init_node("lidar_dataset_recorder")
    LidarDatasetRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
