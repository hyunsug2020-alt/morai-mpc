#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hdmap_viz — 실제 MORAI 주행을 foxglove/rviz로 시각화 (주행 로직엔 관여 안 함, 순수 관찰용).

  표시:
    - HDMAP 전체 차선(회색) + 추월가능구간(녹색, HDMAP 차선변경 허용 링크) + 차선번호 라벨
    - 실제 ego(파랑 큐브 + 현재 차선/좌표 텍스트)
    - 실제 NPC(빨강 큐브)  /Object_topic
    - planner 경로 /avoid_path (노랑 굵은선) ← 차가 이걸 잘 따르는지 관찰

  실행(예): morai.launch(rosbridge) + mpc_lane_avoid.launch(주행) 뜬 뒤
            roslaunch moraimpc viz.launch
"""
import json, math, zipfile
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point, TransformStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
import tf2_ros


MAX_HEADING_DIFF = math.radians(70)
FALLBACK_HEADING_DIFF = math.radians(85)


def yaw_quat(y): return Quaternion(0.0, 0.0, math.sin(y / 2.0), math.cos(y / 2.0))


def norm(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def poly_len(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def point_at_ratio(pts, ratio):
    if len(pts) == 1:
        return pts[0]
    total = poly_len(pts)
    if total <= 1e-6:
        return pts[len(pts) // 2]
    target = total * ratio
    run = 0.0
    for a, b in zip(pts, pts[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg <= 1e-6:
            continue
        if run + seg >= target:
            t = (target - run) / seg
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        run += seg
    return pts[-1]


class HdmapViz:
    def __init__(self):
        rospy.init_node("hdmap_viz")
        zp = rospy.get_param("~hdmap_zip",
            "/mnt/c/Users/Hyunsug2014/Downloads/drive-download-20260701T154301Z-3-001.zip")
        self.show_lane_labels = bool(rospy.get_param("~show_lane_labels", True))
        self.lane_label_min_len = float(rospy.get_param("~lane_label_min_len_m", 8.0))
        self.lane_label_scale = float(rospy.get_param("~lane_label_scale_m", 1.15))
        self.show_ego_pose_label = bool(rospy.get_param("~show_ego_pose_label", True))
        self.show_npc_labels = bool(rospy.get_param("~show_npc_labels", True))
        self.publish_tf = bool(rospy.get_param("~publish_tf", False))
        self._load(zp)
        self.ego = None; self.npcs = []; self.path = []
        self.last_ego_link = None
        self.pub = rospy.Publisher("/viz/markers", MarkerArray, queue_size=1, latch=True)
        self.tfb = tf2_ros.TransformBroadcaster() if self.publish_tf else None
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self._ego_cb, queue_size=1)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self._obj_cb, queue_size=1)
        rospy.Subscriber("/avoid_path", Path, self._path_cb, queue_size=1)
        self._pub_lanes()
        rospy.loginfo("[hdmap_viz] 시각화 ON (link %d, 추월가능 %d)",
                      len(self.links), sum(1 for L in self.links if L["adj"]))

    def _load(self, zp):
        with zipfile.ZipFile(zp) as z:
            links = json.loads(z.read("link_set.json"))
        self.links = []
        for d in links:
            pts = [(p[0], p[1]) for p in d["points"]]
            if len(pts) < 2: continue
            self.links.append({"idx": d.get("idx"),
                               "pts": pts,
                               "lane": d.get("ego_lane"),
                               "road": d.get("road_id"),
                               "length": float(d.get("link_length") or poly_len(pts)),
                               "adj": bool(d.get("can_move_left_lane") or d.get("can_move_right_lane"))})

    def _ego_cb(self, m):
        self.ego = (m.position.x, m.position.y, math.radians(m.heading),
                    math.hypot(m.velocity.x, m.velocity.y))

    def _obj_cb(self, m):
        self.npcs = [(x.position.x, x.position.y, math.radians(x.heading)) for x in m.npc_list]

    def _path_cb(self, m):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    @staticmethod
    def _project(px, py, link):
        best_dist = 1e18
        best_idx = 0
        for idx, (x, y) in enumerate(link["pts"]):
            dist = (px - x) ** 2 + (py - y) ** 2
            if dist < best_dist:
                best_dist = dist
                best_idx = idx
        next_idx = min(best_idx + 1, len(link["pts"]) - 1)
        heading = math.atan2(
            link["pts"][next_idx][1] - link["pts"][best_idx][1],
            link["pts"][next_idx][0] - link["pts"][best_idx][0],
        )
        rx = px - link["pts"][best_idx][0]
        ry = py - link["pts"][best_idx][1]
        lateral = -math.sin(heading) * rx + math.cos(heading) * ry
        return lateral, math.sqrt(best_dist), heading

    def _current_link(self, ex, ey, eh):
        best = None
        best_score = 1e18
        fallback = None
        fallback_dist = 1e18

        for link in self.links:
            if link["lane"] is None:
                continue
            lateral, dist, th = self._project(ex, ey, link)
            if dist > 8.0:
                continue
            heading_diff = abs(norm(th - eh))
            if heading_diff < FALLBACK_HEADING_DIFF and dist < fallback_dist:
                fallback = link
                fallback_dist = dist
            if heading_diff > MAX_HEADING_DIFF:
                continue
            score = abs(lateral) + 0.4 * heading_diff
            if self.last_ego_link == link["idx"]:
                score -= 0.7
            if score < best_score:
                best = link
                best_score = score

        link = best or fallback
        if link is not None:
            self.last_ego_link = link["idx"]
        return link

    def _line(self, ns, mid, color, w):
        mk = Marker(); mk.header.frame_id = "map"; mk.ns = ns; mk.id = mid
        mk.type = Marker.LINE_LIST; mk.action = Marker.ADD; mk.scale.x = w
        mk.color = color; mk.pose.orientation.w = 1.0; return mk

    def _lane_label(self, mid, link):
        x, y = point_at_ratio(link["pts"], 0.50)
        mk = Marker(); mk.header.frame_id = "map"; mk.ns = "lane_labels"; mk.id = mid
        mk.type = Marker.TEXT_VIEW_FACING; mk.action = Marker.ADD
        mk.pose.position = Point(x, y, 1.2); mk.pose.orientation.w = 1.0
        mk.scale.z = self.lane_label_scale
        mk.color = ColorRGBA(0.95, 0.98, 1.0, 0.95)
        mk.text = "%d차선" % int(link["lane"])
        return mk

    def _pub_lanes(self):
        arr = MarkerArray()
        lanes = self._line("lanes", 0, ColorRGBA(0.45, 0.52, 0.60, 0.5), 0.18)
        otz   = self._line("ot_zone", 1, ColorRGBA(0.25, 0.75, 0.35, 0.95), 0.5)
        labels = []
        for i, L in enumerate(self.links):
            tgt = otz if L["adj"] else lanes
            for a, b in zip(L["pts"], L["pts"][1:]):
                tgt.points.append(Point(a[0], a[1], 0.0)); tgt.points.append(Point(b[0], b[1], 0.0))
            if self.show_lane_labels and L["lane"] is not None and L["length"] >= self.lane_label_min_len:
                labels.append(self._lane_label(i, L))
        arr.markers = [lanes, otz] + labels
        self.pub.publish(arr)

    def _pub_dynamic(self):
        now = rospy.Time.now(); arr = MarkerArray()
        if self.ego is not None:
            ex, ey, eh, ev = self.ego
            if self.publish_tf:
                t = TransformStamped()
                t.header.stamp = now; t.header.frame_id = "map"; t.child_frame_id = "base_link"
                t.transform.translation = Vector3(ex, ey, 0.0); t.transform.rotation = yaw_quat(eh)
                self.tfb.sendTransform(t)
            eg = Marker(); eg.header.frame_id = "map"; eg.header.stamp = now
            eg.ns = "ego"; eg.id = 0; eg.type = Marker.CUBE; eg.action = Marker.ADD
            eg.pose.position = Point(ex, ey, 0.75); eg.pose.orientation = yaw_quat(eh)
            eg.scale = Vector3(4.5, 2.0, 1.5); eg.color = ColorRGBA(0.30, 0.60, 1.0, 0.95)
            arr.markers.append(eg)
            tx = Marker(); tx.header.frame_id = "map"; tx.header.stamp = now
            tx.ns = "ego_v"; tx.id = 0; tx.type = Marker.TEXT_VIEW_FACING; tx.action = Marker.ADD
            tx.pose.position = Point(ex, ey, 3.2); tx.scale.z = 1.1
            tx.color = ColorRGBA(1, 1, 1, 0.95)
            if self.show_ego_pose_label:
                link = self._current_link(ex, ey, eh)
                lane_text = "%d차선 road=%s" % (int(link["lane"]), str(link["road"])) if link else "차선 미확인"
                tx.text = "EGO\n%s\n%.0f km/h\nx=%.1f y=%.1f" % (lane_text, ev * 3.6, ex, ey)
            else:
                tx.text = "%.0f km/h" % (ev * 3.6)
            arr.markers.append(tx)
        if self.path:
            pm = Marker(); pm.header.frame_id = "map"; pm.header.stamp = now
            pm.ns = "path"; pm.id = 0; pm.type = Marker.LINE_STRIP; pm.action = Marker.ADD
            pm.scale.x = 0.45; pm.color = ColorRGBA(1.0, 0.80, 0.10, 0.95); pm.pose.orientation.w = 1.0
            for x, y in self.path: pm.points.append(Point(x, y, 0.15))
            arr.markers.append(pm)
        for i, (x, y, h) in enumerate(self.npcs):
            nm = Marker(); nm.header.frame_id = "map"; nm.header.stamp = now
            nm.ns = "npc"; nm.id = i; nm.type = Marker.CUBE; nm.action = Marker.ADD
            nm.pose.position = Point(x, y, 0.75); nm.pose.orientation = yaw_quat(h)
            nm.scale = Vector3(4.5, 2.0, 1.5); nm.color = ColorRGBA(1.0, 0.40, 0.30, 0.95)
            nm.lifetime = rospy.Duration(0.5)
            arr.markers.append(nm)
            if self.show_npc_labels:
                nt = Marker(); nt.header.frame_id = "map"; nt.header.stamp = now
                nt.ns = "npc_label"; nt.id = i; nt.type = Marker.TEXT_VIEW_FACING; nt.action = Marker.ADD
                nt.pose.position = Point(x, y, 3.0); nt.pose.orientation.w = 1.0
                nt.scale.z = 1.0; nt.color = ColorRGBA(1.0, 0.86, 0.78, 0.95)
                nt.text = "NPC%d" % i
                nt.lifetime = rospy.Duration(0.5)
                arr.markers.append(nt)
        self.pub.publish(arr)

    def spin(self):
        r = rospy.Rate(20); k = 0
        while not rospy.is_shutdown():
            self._pub_dynamic()
            k += 1
            if k % 40 == 0: self._pub_lanes()          # 차선 주기적 재발행(늦게 접속한 foxglove 대비)
            r.sleep()


if __name__ == "__main__":
    try: HdmapViz().spin()
    except rospy.ROSInterruptException: pass
