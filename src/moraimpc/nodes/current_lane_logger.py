#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import zipfile

import rospy
from morai_msgs.msg import EgoVehicleStatus

MAX_HEADING_DIFF = math.radians(70)
FALLBACK_HEADING_DIFF = math.radians(85)


def norm(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Link:
    __slots__ = ("idx", "pts", "lane", "road")

    def __init__(self, data):
        self.idx = data["idx"]
        self.pts = [(pt[0], pt[1]) for pt in data["points"]]
        self.lane = data.get("ego_lane")
        self.road = data.get("road_id")


class CurrentLaneLogger:
    def __init__(self):
        rospy.init_node("current_lane_logger")
        hdmap_zip = rospy.get_param("~hdmap_zip", "$(find moraimpc)/data/hdmap.zip")
        # rosparam does not resolve $(find ...) inside Python, so keep a safe package-local fallback.
        if "$(find" in hdmap_zip:
            hdmap_zip = "/home/coss/catkin_ws/src/moraimpc/data/hdmap.zip"

        self.links = self._load_links(hdmap_zip)
        self.last_link_idx = None
        self.last_lane = None
        self.last_log_time = 0.0
        self.log_period = float(rospy.get_param("~log_period_sec", 2.0))

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self._ego_cb, queue_size=1)
        rospy.loginfo("[LaneLogger] 시작됨 - HD map link %d개 로드", len(self.links))

    def _load_links(self, hdmap_zip):
        with zipfile.ZipFile(hdmap_zip) as zf:
            links = json.loads(zf.read("link_set.json"))
        out = []
        for data in links:
            link = Link(data)
            if len(link.pts) >= 2 and link.lane is not None:
                out.append(link)
        return out

    @staticmethod
    def _project(px, py, link):
        best_dist = 1e18
        best_idx = 0
        for idx, (x, y) in enumerate(link.pts):
            dist = (px - x) ** 2 + (py - y) ** 2
            if dist < best_dist:
                best_dist = dist
                best_idx = idx
        next_idx = min(best_idx + 1, len(link.pts) - 1)
        heading = math.atan2(
            link.pts[next_idx][1] - link.pts[best_idx][1],
            link.pts[next_idx][0] - link.pts[best_idx][0],
        )
        rx = px - link.pts[best_idx][0]
        ry = py - link.pts[best_idx][1]
        lateral = -math.sin(heading) * rx + math.cos(heading) * ry
        return lateral, math.sqrt(best_dist), heading

    def _current_link(self, ex, ey, eh):
        best = None
        best_score = 1e18
        best_heading_diff = None
        fallback = None
        fallback_dist = 1e18
        fallback_heading_diff = None

        for link in self.links:
            lateral, dist, th = self._project(ex, ey, link)
            if dist > 8.0:
                continue
            heading_diff = abs(norm(th - eh))
            if heading_diff < FALLBACK_HEADING_DIFF and dist < fallback_dist:
                fallback = link
                fallback_dist = dist
                fallback_heading_diff = heading_diff
            if heading_diff > MAX_HEADING_DIFF:
                continue
            score = abs(lateral) + 0.4 * heading_diff
            if self.last_link_idx == link.idx:
                score -= 0.7
            if score < best_score:
                best = link
                best_score = score
                best_heading_diff = heading_diff

        if best is not None:
            return best, best_heading_diff
        return fallback, fallback_heading_diff

    def _ego_cb(self, msg):
        ex = msg.position.x
        ey = msg.position.y
        eh = math.radians(msg.heading)
        link, heading_diff = self._current_link(ex, ey, eh)
        if link is None:
            rospy.logwarn_throttle(2.0, "[LaneLogger] 현재 위치에서 차선을 찾지 못했음")
            return

        now = rospy.get_time()
        changed = (link.idx != self.last_link_idx) or (link.lane != self.last_lane)
        periodic = (now - self.last_log_time) >= self.log_period

        if changed or periodic:
            rospy.loginfo(
                "[LaneLogger] 현재 %d차선입니다. road_id=%s link=%s heading_diff=%.1fdeg position=(%.2f, %.2f)",
                int(link.lane), str(link.road), link.idx, math.degrees(heading_diff or 0.0), ex, ey
            )
            self.last_log_time = now

        self.last_link_idx = link.idx
        self.last_lane = link.lane


if __name__ == "__main__":
    try:
        CurrentLaneLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
