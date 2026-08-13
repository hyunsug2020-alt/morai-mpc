#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import math
import sys
import zipfile

import rospy
from morai_msgs.msg import EgoVehicleStatus
from std_msgs.msg import String

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


class AutoAvoidTestDriver:
    def __init__(self):
        rospy.init_node("auto_avoid_test_driver")
        hdmap_zip = rospy.get_param("~hdmap_zip", "/home/david/morai-mpc-agent-morai-lio-gps-integration/src/moraimpc/data/hdmap.zip")
        self.links = self._load_links(hdmap_zip)
        self.expected = self._parse_sequence(rospy.get_param("~expected_sequence", "1,2,1"))
        self.result_file = rospy.get_param("~result_file", "/tmp/auto_avoid_test_result.json")
        self.timeout = float(rospy.get_param("~timeout_sec", 90.0))

        self.ego = None
        self.status = ""
        self.last_link_idx = None
        self.history = []
        self.progress = 0
        self.start_time = rospy.get_time()

        rospy.Subscriber("/localization/ego_status",
                         EgoVehicleStatus, self._ego_cb, queue_size=1)
        rospy.Subscriber("/lane_change_status", String, self._status_cb, queue_size=5)
        rospy.loginfo("[AutoAvoidTest] expected=%s", self.expected)

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
            return None
        ex = self.ego.position.x
        ey = self.ego.position.y
        eh = math.radians(self.ego.heading)
        best = None
        best_score = 1e18
        fallback = None
        fallback_dist = 1e18
        for link in self.links:
            lat, dist, th = self._project(ex, ey, link)
            if dist > 8.0:
                continue
            hd = abs(norm(th - eh))
            if hd < FALLBACK_HEADING_DIFF and dist < fallback_dist:
                fallback = link
                fallback_dist = dist
            if hd > MAX_HEADING_DIFF:
                continue
            score = abs(lat) + 0.4 * hd
            if self.last_link_idx == link.idx:
                score -= 0.7
            if score < best_score:
                best = link
                best_score = score
        link = best or fallback
        if link is not None:
            self.last_link_idx = link.idx
        return link

    def _record(self, link):
        if link is None:
            return
        if not self.history or self.history[-1]["lane"] != int(link.lane):
            item = {
                "t": round(rospy.get_time() - self.start_time, 2),
                "lane": int(link.lane),
                "road_id": str(link.road),
                "link": link.idx,
                "status": self.status,
            }
            self.history.append(item)
            if self.progress < len(self.expected) and item["lane"] == self.expected[self.progress]:
                self.progress += 1

    def _write_result(self, success, reason):
        result = {
            "success": bool(success),
            "reason": reason,
            "expected": self.expected,
            "progress": self.progress,
            "history": self.history,
            "last_status": self.status,
            "elapsed": round(rospy.get_time() - self.start_time, 2),
        }
        with open(self.result_file, "w") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        rospy.loginfo("[AutoAvoidTest] RESULT %s", json.dumps(result, ensure_ascii=False))

    def spin(self):
        rate = rospy.Rate(10)
        while self.ego is None and not rospy.is_shutdown():
            rate.sleep()
        while not rospy.is_shutdown():
            self._record(self._current_link())
            if self.progress >= len(self.expected):
                # Give the planner a short moment to settle back to cruise.
                if self.status.startswith("AUTO_CRUISE") or rospy.get_time() - self.start_time > 8.0:
                    self._write_result(True, "completed")
                    rospy.signal_shutdown("auto avoid completed")
                    return
            if rospy.get_time() - self.start_time > self.timeout:
                self._write_result(False, "timeout")
                sys.exit(2)
            rate.sleep()


if __name__ == "__main__":
    try:
        AutoAvoidTestDriver().spin()
    except rospy.ROSInterruptException:
        pass
