#!/usr/bin/env python3
"""Stop a dataset recording session at its frame or wall-time target."""

import json
import math
import os
import threading
import time
from datetime import datetime

import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool


class DatasetRecordingGuard:
    def __init__(self):
        self.status_topic = rospy.get_param(
            "~status_topic", "/lidar_detection/dataset_status"
        )
        self.guard_status_topic = rospy.get_param(
            "~guard_status_topic",
            "/lidar_detection/collection_guard_status",
        )
        self.service_name = rospy.get_param(
            "~recorder_service", "/lidar_dataset_recorder/set_enabled"
        )
        self.diagnostic_topic = rospy.get_param(
            "~diagnostic_topic",
            "/lidar_detection/ground_truth_diagnostics",
        )
        self.target_total_frames = max(
            1, int(rospy.get_param("~target_total_frames", 11000))
        )
        self.max_duration_seconds = max(
            1.0, float(rospy.get_param("~max_duration_seconds", 3600.0))
        )
        self.check_period_seconds = max(
            1.0, float(rospy.get_param("~check_period_seconds", 5.0))
        )
        self.auto_quality_gate = bool(
            rospy.get_param("~auto_quality_gate", False)
        )
        self.min_points_per_box = max(
            0, int(rospy.get_param("~min_points_per_box", 5))
        )
        self.max_label_range_m = max(
            0.0, float(rospy.get_param("~max_label_range_m", 70.0))
        )
        useful_category = rospy.get_param("~useful_category", None)
        if useful_category:
            # Prefer the singular launch parameter.  This also prevents a
            # stale ~useful_categories value left on the parameter server by
            # an older launch from silently overriding the current session.
            self.useful_categories = {str(useful_category)}
        else:
            self.useful_categories = {
                str(category)
                for category in rospy.get_param(
                    "~useful_categories", ["npc"]
                )
            }
        self.resume_consecutive_samples = max(
            1, int(rospy.get_param("~resume_consecutive_samples", 25))
        )
        self.pause_after_no_useful_seconds = max(
            1.0,
            float(
                rospy.get_param(
                    "~pause_after_no_useful_seconds",
                    5.0,
                )
            ),
        )

        self.lock = threading.Lock()
        self.latest_status = None
        self.status_revision = 0
        self.latest_quality = None
        self.useful_streak = 0
        self.last_useful_monotonic = None
        self.last_diagnostic_monotonic = None
        self.started_monotonic = time.monotonic()
        self.last_timer_monotonic = self.started_monotonic
        self.active_elapsed_seconds = 0.0
        self.started_at = datetime.now().astimezone().isoformat()
        self.finished = False
        self.status_pub = rospy.Publisher(
            self.guard_status_topic, String, queue_size=1, latch=True
        )
        self.status_sub = rospy.Subscriber(
            self.status_topic, String, self.status_callback, queue_size=5
        )
        self.diagnostic_sub = rospy.Subscriber(
            self.diagnostic_topic,
            String,
            self.diagnostic_callback,
            queue_size=20,
        )
        self.timer = rospy.Timer(
            rospy.Duration(self.check_period_seconds), self.timer_callback
        )
        rospy.loginfo(
            "Dataset guard active: target=%d frames max_duration=%.0fs "
            "auto_quality_gate=%s",
            self.target_total_frames,
            self.max_duration_seconds,
            self.auto_quality_gate,
        )

    def status_callback(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError) as error:
            rospy.logwarn_throttle(
                2.0, "Invalid dataset recorder status: %s", error
            )
            return
        with self.lock:
            self.latest_status = status
            self.status_revision += 1

    def diagnostic_callback(self, message):
        try:
            diagnostic = json.loads(message.data)
            useful_objects = []
            for obj in diagnostic.get("objects", []):
                if str(obj.get("category", "")) not in self.useful_categories:
                    continue
                center = obj.get("center", [])
                if len(center) < 2:
                    continue
                distance = math.hypot(float(center[0]), float(center[1]))
                points_in_box = int(obj.get("points_in_box", 0))
                if (
                    points_in_box >= self.min_points_per_box
                    and (
                        self.max_label_range_m <= 0.0
                        or distance <= self.max_label_range_m
                    )
                ):
                    useful_objects.append(
                        {
                            "id": int(obj.get("id", -1)),
                            "distance_m": round(distance, 3),
                            "points_in_box": points_in_box,
                        }
                    )
        except (TypeError, ValueError) as error:
            rospy.logwarn_throttle(
                2.0, "Invalid ground-truth diagnostic for guard: %s", error
            )
            return

        now = time.monotonic()
        with self.lock:
            if useful_objects:
                self.useful_streak += 1
                self.last_useful_monotonic = now
            else:
                self.useful_streak = 0
            self.last_diagnostic_monotonic = now
            self.latest_quality = {
                "stamp_ns": diagnostic.get("stamp_ns"),
                "useful_objects": len(useful_objects),
                "useful_streak": self.useful_streak,
                "objects": useful_objects,
            }

    def timer_callback(self, _event):
        if self.finished:
            return
        now = time.monotonic()
        wall_elapsed = now - self.started_monotonic
        with self.lock:
            status = (
                None
                if self.latest_status is None
                else dict(self.latest_status)
            )
            quality = (
                None
                if self.latest_quality is None
                else dict(self.latest_quality)
            )
            useful_streak = self.useful_streak
            last_useful = self.last_useful_monotonic
            last_diagnostic = self.last_diagnostic_monotonic
            if status is not None and bool(status.get("enabled", False)):
                self.active_elapsed_seconds += (
                    now - self.last_timer_monotonic
                )
            self.last_timer_monotonic = now
            active_elapsed = self.active_elapsed_seconds

        quality_gate_action = None
        recorder_enabled = bool(
            status is not None and status.get("enabled", False)
        )
        diagnostic_age = (
            None if last_diagnostic is None else now - last_diagnostic
        )
        if (
            self.auto_quality_gate
            and status is not None
            and diagnostic_age is not None
            and diagnostic_age <= max(2.0, self.check_period_seconds * 2.0)
        ):
            no_useful_seconds = (
                wall_elapsed
                if last_useful is None
                else now - last_useful
            )
            if (
                recorder_enabled
                and no_useful_seconds
                >= self.pause_after_no_useful_seconds
            ):
                if self.set_recorder_enabled(False):
                    recorder_enabled = False
                    status["enabled"] = False
                    quality_gate_action = "paused_no_useful_object"
            elif (
                not recorder_enabled
                and useful_streak >= self.resume_consecutive_samples
            ):
                if self.set_recorder_enabled(True):
                    recorder_enabled = True
                    status["enabled"] = True
                    quality_gate_action = "resumed_useful_object"

        next_index = 0 if status is None else int(status.get("next_index", 0))
        reason = None
        if next_index >= self.target_total_frames:
            reason = "target_frames_reached"
        elif active_elapsed >= self.max_duration_seconds:
            reason = "max_duration_reached"

        progress = {
            "active": reason is None,
            "elapsed_seconds": round(active_elapsed, 3),
            "wall_elapsed_seconds": round(wall_elapsed, 3),
            "target_total_frames": self.target_total_frames,
            "current_total_frames": next_index,
            "progress_percent": round(
                min(100.0, 100.0 * next_index / self.target_total_frames),
                3,
            ),
            "max_duration_seconds": self.max_duration_seconds,
            "recorder": status,
            "quality_gate": {
                "enabled": self.auto_quality_gate,
                "useful_categories": sorted(self.useful_categories),
                "recorder_enabled": recorder_enabled,
                "action": quality_gate_action,
                "diagnostic_age_seconds": (
                    None
                    if diagnostic_age is None
                    else round(diagnostic_age, 3)
                ),
                "resume_consecutive_samples": (
                    self.resume_consecutive_samples
                ),
                "pause_after_no_useful_seconds": (
                    self.pause_after_no_useful_seconds
                ),
                "latest": quality,
            },
            "finish_reason": reason,
        }
        self.status_pub.publish(
            String(
                data=json.dumps(
                    progress, ensure_ascii=False, separators=(",", ":")
                )
            )
        )
        if reason is not None:
            self.finish_collection(reason, progress)
        else:
            rospy.loginfo_throttle(
                60.0,
                "Dataset collection %.1f%%: %d/%d frames elapsed=%.1fmin",
                progress["progress_percent"],
                next_index,
                self.target_total_frames,
                active_elapsed / 60.0,
            )

    def set_recorder_enabled(self, enabled):
        try:
            rospy.wait_for_service(self.service_name, timeout=2.0)
            response = rospy.ServiceProxy(self.service_name, SetBool)(enabled)
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn_throttle(
                2.0,
                "Could not set dataset recorder enabled=%s: %s",
                enabled,
                error,
            )
            return False
        if not response.success:
            rospy.logwarn(
                "Dataset recorder rejected enabled=%s: %s",
                enabled,
                response.message,
            )
            return False
        with self.lock:
            if self.latest_status is not None:
                self.latest_status["enabled"] = enabled
        rospy.loginfo(
            "Dataset quality gate set recorder enabled=%s: %s",
            enabled,
            response.message,
        )
        return True

    def finish_collection(self, reason, progress):
        self.finished = True
        pre_stop_frames = int(progress.get("current_total_frames", 0))
        stop_success = self.set_recorder_enabled(False)
        stop_message = (
            "recording paused"
            if stop_success
            else "could not pause recorder"
        )
        final_status = self.wait_for_recorder_drain()
        if final_status is not None:
            final_frames = int(
                final_status.get("next_index", pre_stop_frames)
            )
            progress["recorder"] = final_status
            progress["current_total_frames"] = final_frames
            progress["progress_percent"] = round(
                min(
                    100.0,
                    100.0 * final_frames / self.target_total_frames,
                ),
                3,
            )
            progress["post_stop_frames_flushed"] = max(
                0, final_frames - pre_stop_frames
            )
            progress["quality_gate"]["recorder_enabled"] = bool(
                final_status.get("enabled", False)
            )

        progress["active"] = False
        progress["finish_reason"] = reason
        progress["finished_at"] = datetime.now().astimezone().isoformat()
        progress["started_at"] = self.started_at
        progress["stop_service_success"] = stop_success
        progress["stop_service_message"] = stop_message
        self.write_report(progress)
        self.status_pub.publish(
            String(
                data=json.dumps(
                    progress, ensure_ascii=False, separators=(",", ":")
                )
            )
        )
        rospy.loginfo(
            "Dataset collection finished: reason=%s frames=%d",
            reason,
            progress["current_total_frames"],
        )
        rospy.signal_shutdown("dataset collection complete")

    def wait_for_recorder_drain(self):
        deadline = time.monotonic() + 5.0
        last_revision = -1
        last_index = None
        stable_updates = 0
        final_status = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                revision = self.status_revision
                status = (
                    None
                    if self.latest_status is None
                    else dict(self.latest_status)
                )
            if status is not None:
                final_status = status
            if revision != last_revision and status is not None:
                last_revision = revision
                index = int(status.get("next_index", 0))
                drained = (
                    not bool(status.get("enabled", True))
                    and int(status.get("writer_queue", 1)) == 0
                )
                if drained and index == last_index:
                    stable_updates += 1
                elif drained:
                    stable_updates = 1
                else:
                    stable_updates = 0
                last_index = index
                if stable_updates >= 2:
                    break
            time.sleep(0.1)
        return final_status

    @staticmethod
    def write_report(progress):
        recorder = progress.get("recorder") or {}
        output_dir = recorder.get("output_dir")
        if not output_dir or not os.path.isdir(output_dir):
            return
        report_path = os.path.join(output_dir, "collection_report.json")
        temporary = report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(
                progress,
                stream,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, report_path)


def main():
    rospy.init_node("dataset_recording_guard")
    DatasetRecordingGuard()
    rospy.spin()


if __name__ == "__main__":
    main()
