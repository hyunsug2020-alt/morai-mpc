#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import math
import sys
import zipfile

import rospy
from morai_msgs.msg import EgoVehicleStatus
from std_msgs.msg import Int32, String

MAX_HEADING_DIFF = math.radians(70)
FALLBACK_HEADING_DIFF = math.radians(85)


def norm(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Link:
    __slots__ = ("idx", "pts", "lane", "road")

    def __init__(self, data):
        self.idx = data["idx"]
        self.pts = [(p[0], p[1]) for p in data["points"]]
        self.lane = data.get("ego_lane")
        self.road = data.get("road_id")


class LaneChangeTestDriver:
    def __init__(self):
        rospy.init_node("lane_change_test_driver")
        hdmap_zip = rospy.get_param("~hdmap_zip", "/home/coss/catkin_ws/src/moraimpc/data/hdmap.zip")
        self.links = self._load_links(hdmap_zip)
        self.sequence = self._parse_sequence(rospy.get_param("~sequence", "2,3,2,1"))
        self.result_file = rospy.get_param("~result_file", "/tmp/lane_change_test_result.json")
        self.per_target_timeout = float(rospy.get_param("~per_target_timeout_sec", 16.0))
        self.overall_timeout = float(rospy.get_param("~overall_timeout_sec", 90.0))
        self.hold_sec = float(rospy.get_param("~arrive_hold_sec", 0.8))
        self.arrive_lat_m = float(rospy.get_param("~arrive_lat_m", 1.1))

        self.pub_cmd = rospy.Publisher("/lane_change_cmd", Int32, queue_size=1, latch=True)
        rospy.Subscriber("/localization/ego_status",
                         EgoVehicleStatus, self._ego_cb, queue_size=1)
        rospy.Subscriber("/lane_change_status", String, self._status_cb, queue_size=5)

        self.ego = None
        self.status = ""
        self.last_link_idx = None
        self.step = -1
        self.step_start = None
        self.target_reached_since = None
        self.start_time = rospy.get_time()
        self.history = []
        self.max_abs_lat = 0.0

        rospy.loginfo("[LaneChangeTest] sequence=%s", self.sequence)

    @staticmethod
    def _parse_sequence(raw):
        if isinstance(raw, list):
            return [int(x) for x in raw]
        raw = str(raw).strip()
        if raw.startswith("["):
            return [int(x) for x in json.loads(raw)]
        return [int(x.strip()) for x in raw.split(",") if x.strip()]

    @staticmethod
    def _load_links(hdmap_zip):
        with zipfile.ZipFile(hdmap_zip) as zf:
            raw_links = json.loads(zf.read("link_set.json"))
        out = []
        for data in raw_links:
            link = Link(data)
            if len(link.pts) >= 2 and link.lane is not None:
                out.append(link)
        return out

    def _ego_cb(self, msg):
        self.ego = msg

    def _status_cb(self, msg):
        self.status = msg.data

    @staticmethod
    def _project(px, py, link):
        best_dist = 1e18
        best_idx = 0
        for i, (x, y) in enumerate(link.pts):
            dist = (px - x) ** 2 + (py - y) ** 2
            if dist < best_dist:
                best_dist = dist
                best_idx = i
        next_idx = min(best_idx + 1, len(link.pts) - 1)
        th = math.atan2(
            link.pts[next_idx][1] - link.pts[best_idx][1],
            link.pts[next_idx][0] - link.pts[best_idx][0],
        )
        rx = px - link.pts[best_idx][0]
        ry = py - link.pts[best_idx][1]
        lat = -math.sin(th) * rx + math.cos(th) * ry
        return lat, math.sqrt(best_dist), th

    def _current_link(self):
        if self.ego is None:
            return None, None
        ex = self.ego.position.x
        ey = self.ego.position.y
        eh = math.radians(self.ego.heading)
        best = None
        best_score = 1e18
        fallback = None
        fallback_dist = 1e18
        best_lat = None
        fallback_lat = None
        for link in self.links:
            lat, dist, th = self._project(ex, ey, link)
            if dist > 8.0:
                continue
            hd = abs(norm(th - eh))
            if hd < FALLBACK_HEADING_DIFF and dist < fallback_dist:
                fallback = link
                fallback_dist = dist
                fallback_lat = lat
            if hd > MAX_HEADING_DIFF:
                continue
            score = abs(lat) + 0.4 * hd
            if self.last_link_idx == link.idx:
                score -= 0.7
            if score < best_score:
                best_score = score
                best = link
                best_lat = lat
        link = best if best is not None else fallback
        lat = best_lat if best is not None else fallback_lat
        if link is not None:
            self.last_link_idx = link.idx
        return link, lat

    def _write_result(self, success, reason):
        result = {
            "success": bool(success),
            "reason": reason,
            "sequence": self.sequence,
            "history": self.history,
            "max_abs_lat": round(self.max_abs_lat, 3),
            "last_status": self.status,
            "elapsed": round(rospy.get_time() - self.start_time, 2),
        }
        with open(self.result_file, "w") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        rospy.loginfo("[LaneChangeTest] RESULT %s", json.dumps(result, ensure_ascii=False))

    def _send_next(self):
        self.step += 1
        if self.step >= len(self.sequence):
            self.pub_cmd.publish(Int32(data=0))
            self._write_result(True, "completed")
            rospy.signal_shutdown("lane change test completed")
            return
        target = self.sequence[self.step]
        self.step_start = rospy.get_time()
        self.target_reached_since = None
        self.pub_cmd.publish(Int32(data=target))
        rospy.loginfo("[LaneChangeTest] command target lane=%d", target)

    def spin(self):
        rate = rospy.Rate(10)
        while self.ego is None and not rospy.is_shutdown():
            rate.sleep()
        self._send_next()

        while not rospy.is_shutdown():
            now = rospy.get_time()
            if now - self.start_time > self.overall_timeout:
                self._write_result(False, "overall-timeout")
                sys.exit(2)

            link, lat = self._current_link()
            if link is not None and lat is not None:
                self.max_abs_lat = max(self.max_abs_lat, abs(lat))
                sample = {
                    "t": round(now - self.start_time, 2),
                    "lane": int(link.lane),
                    "road_id": str(link.road),
                    "link": link.idx,
                    "lat": round(lat, 3),
                    "target": self.sequence[self.step] if self.step < len(self.sequence) else None,
                }
                if not self.history or self.history[-1]["lane"] != sample["lane"]:
                    self.history.append(sample)

                target = self.sequence[self.step] if self.step < len(self.sequence) else None
                if target is not None and link.lane == target and abs(lat) < self.arrive_lat_m:
                    if self.target_reached_since is None:
                        self.target_reached_since = now
                    elif now - self.target_reached_since >= self.hold_sec:
                        self._send_next()
                else:
                    self.target_reached_since = None

            if self.step_start is not None and now - self.step_start > self.per_target_timeout:
                target = self.sequence[self.step] if self.step < len(self.sequence) else None
                self._write_result(False, "target-timeout-%s" % target)
                sys.exit(2)

            rate.sleep()


if __name__ == "__main__":
    try:
        LaneChangeTestDriver().spin()
    except rospy.ROSInterruptException:
        pass
