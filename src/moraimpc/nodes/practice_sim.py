#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""practice_sim — MORAI SIM 없이 HDMAP 위에서 추월 동작을 관찰하는 경량 시뮬레이터.

  - ego를 HDMAP route 시작점에 놓고, planner(/avoid_path)를 pure-pursuit로 추종시켜 실제 주행.
  - route를 따라 15km/h로 느리게 가는 가상차(NPC)를 일정간격 배치 → ego가 만나면 추월 시도.
  - ego/NPC/차선/추월가능구간을 MarkerArray + TF로 발행 → foxglove(rosbridge 9090)/rviz서 관찰.

  실행: roslaunch moraimpc practic.launch   (morai.launch와 동시 실행 금지 — /Ego_topic 충돌)
"""
import json, math, zipfile
from collections import defaultdict

import rospy
from std_msgs.msg import Float32, ColorRGBA
from geometry_msgs.msg import Vector3, Point, TransformStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList, ObjectStatus
import tf2_ros


def norm(a): return (a + math.pi) % (2 * math.pi) - math.pi
def yaw_quat(y): return Quaternion(0.0, 0.0, math.sin(y / 2.0), math.cos(y / 2.0))


class PracticeSim:
    def __init__(self):
        rospy.init_node("practice_sim")
        zp = rospy.get_param("~hdmap_zip",
            "/mnt/c/Users/Hyunsug2014/Downloads/drive-download-20260701T154301Z-3-001.zip")
        self.npc_v   = rospy.get_param("~npc_speed_kmh", 15.0) / 3.6   # 가상차 속도
        self.cruise  = rospy.get_param("~cruise_mps", 11.0)
        self.route_m = rospy.get_param("~route_len_m", 500.0)
        self.npc_gap = rospy.get_param("~npc_spacing_m", 90.0)
        self.dt      = 0.05

        self._load(zp)
        self._build_route()
        self._place_npcs()

        self.ex, self.ey = self.route[0]
        self.eh = math.atan2(self.route[6][1] - self.route[0][1], self.route[6][0] - self.route[0][0])
        self.ev = 3.0
        self.path = []; self.tgt_v = None

        self.pub_ego = rospy.Publisher("/Ego_topic", EgoVehicleStatus, queue_size=1)
        self.pub_obj = rospy.Publisher("/Object_topic", ObjectStatusList, queue_size=1)
        self.pub_mk  = rospy.Publisher("/practice/markers", MarkerArray, queue_size=1, latch=True)
        self.tfb = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/avoid_path", Path, self._path_cb, queue_size=1)
        rospy.Subscriber("/avoid_target_vel", Float32, lambda m: setattr(self, "tgt_v", m.data), queue_size=1)

        self._pub_static_markers()
        rospy.loginfo("[practice] route %.0fm, NPC %d대(%.0fkm/h), ego cruise %.1f m/s",
                      self.route_s[-1], len(self.npcs), self.npc_v * 3.6, self.cruise)

    # ── HDMAP ──
    def _load(self, zp):
        with zipfile.ZipFile(zp) as z:
            links = json.loads(z.read("link_set.json"))
        self.links = {}; self.adj = defaultdict(list)
        for d in links:
            pts = [(p[0], p[1]) for p in d["points"]]
            if len(pts) < 2: continue
            L = {"idx": d["idx"], "fr": d["from_node_idx"], "to": d["to_node_idx"], "pts": pts,
                 "lane": d.get("ego_lane"), "can_l": bool(d.get("can_move_left_lane")),
                 "can_r": bool(d.get("can_move_right_lane")), "width": d.get("width_start") or 3.5,
                 "length": d.get("link_length") or sum(math.hypot(b[0]-a[0], b[1]-a[1])
                                                       for a, b in zip(pts, pts[1:]))}
            self.links[L["idx"]] = L; self.adj[L["fr"]].append(L)

    @staticmethod
    def _hd(L):
        return math.atan2(L["pts"][1][1] - L["pts"][0][1], L["pts"][1][0] - L["pts"][0][0])

    def _forward(self, start, length):
        chain = [start]; tot = start["length"]; used = {start["idx"]}; cur = start
        while tot < length and len(chain) < 300:
            nx = [L for L in self.adj.get(cur["to"], []) if L["idx"] not in used]
            if not nx: break
            eh = math.atan2(cur["pts"][-1][1] - cur["pts"][-2][1], cur["pts"][-1][0] - cur["pts"][-2][0])
            n = min(nx, key=lambda L: abs(norm(self._hd(L) - eh)))
            if abs(norm(self._hd(n) - eh)) > math.radians(50): break
            chain.append(n); used.add(n["idx"]); tot += n["length"]; cur = n
        return chain

    def _build_route(self):
        # 추월가능(can_l/can_r) 링크를 가장 많이 지나는 전방 체인을 route로 선택
        best = None; bs = -1
        for st in self.links.values():
            if st["length"] < 10: continue
            ch = self._forward(st, self.route_m)
            if sum(l["length"] for l in ch) < self.route_m * 0.6: continue
            score = sum(1 for l in ch if l["can_l"] or l["can_r"])
            if score > bs: bs = score; best = ch
        if best is None: best = self._forward(next(iter(self.links.values())), self.route_m)
        # 폴리라인 + 누적 s
        pts = []
        for L in best:
            for p in L["pts"]:
                if pts and (pts[-1][0]-p[0])**2 + (pts[-1][1]-p[1])**2 < 0.04: continue
                pts.append(p)
        self.route = self._resample(pts, 1.0)
        self.route_s = [0.0]
        for i in range(1, len(self.route)):
            self.route_s.append(self.route_s[-1] +
                math.hypot(self.route[i][0]-self.route[i-1][0], self.route[i][1]-self.route[i-1][1]))
        self.route_links = best

    @staticmethod
    def _resample(pts, step):
        if len(pts) < 2: return list(pts)
        out = [pts[0]]; carry = 0.0
        for i in range(1, len(pts)):
            ax, ay = out[-1]; bx, by = pts[i]; seg = math.hypot(bx-ax, by-ay)
            if seg < 1e-9: continue
            d = seg; sx, sy = ax, ay
            while carry + d >= step:
                t = (step - carry) / d; nx, ny = sx+(bx-sx)*t, sy+(by-sy)*t
                out.append((nx, ny)); sx, sy = nx, ny; d = math.hypot(bx-sx, by-sy); carry = 0.0
            carry += d
        out.append(pts[-1]); return out

    def _route_pose(self, s):
        s = max(0.0, min(self.route_s[-1], s))
        lo, hi = 0, len(self.route_s) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.route_s[mid] < s: lo = mid + 1
            else: hi = mid
        i = max(0, lo - 1); j = min(i + 1, len(self.route) - 1)
        seg = max(self.route_s[j] - self.route_s[i], 1e-6)
        t = (s - self.route_s[i]) / seg
        x = self.route[i][0] + (self.route[j][0]-self.route[i][0]) * t
        y = self.route[i][1] + (self.route[j][1]-self.route[i][1]) * t
        h = math.atan2(self.route[j][1]-self.route[i][1], self.route[j][0]-self.route[i][0])
        return x, y, h

    def _ego_s(self):
        ci = min(range(len(self.route)), key=lambda i: (self.route[i][0]-self.ex)**2 + (self.route[i][1]-self.ey)**2)
        return self.route_s[ci]

    def _place_npcs(self):
        self.npcs = []
        s = self.npc_gap
        while s < self.route_s[-1] - 40.0:
            self.npcs.append({"s": s}); s += self.npc_gap

    # ── 콜백/스텝 ──
    def _path_cb(self, m):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    def _drive(self):
        dt = self.dt
        if self.path and len(self.path) >= 2:
            ci = min(range(len(self.path)), key=lambda i: (self.path[i][0]-self.ex)**2 + (self.path[i][1]-self.ey)**2)
            tx, ty = self.path[-1]; acc = 0.0
            for i in range(ci, len(self.path) - 1):
                acc += math.hypot(self.path[i+1][0]-self.path[i][0], self.path[i+1][1]-self.path[i][1])
                if acc >= 4.5: tx, ty = self.path[i+1]; break
            dh = norm(math.atan2(ty - self.ey, tx - self.ex) - self.eh)
            self.eh += max(-1.4*dt, min(1.4*dt, dh))
        tv = self.tgt_v if self.tgt_v is not None else self.cruise
        self.ev += max(-5.0*dt, min(3.0*dt, tv - self.ev))
        self.ev = max(0.0, min(self.cruise*1.6, self.ev))
        self.ex += self.ev * math.cos(self.eh) * dt
        self.ey += self.ev * math.sin(self.eh) * dt
        # NPC 전진
        for npc in self.npcs:
            npc["s"] += self.npc_v * dt
        # ego가 끝에 도달 → 전체 리셋(반복 관찰)
        if self._ego_s() > self.route_s[-1] - 12.0:
            self.ex, self.ey = self.route[0]
            self.eh = math.atan2(self.route[6][1]-self.route[0][1], self.route[6][0]-self.route[0][0])
            self.ev = 3.0; self._place_npcs()

    def _publish(self):
        now = rospy.Time.now()
        # ego
        e = EgoVehicleStatus()
        e.header.stamp = now; e.header.frame_id = "map"
        e.position = Vector3(self.ex, self.ey, 0.0)
        e.velocity = Vector3(self.ev*math.cos(self.eh), self.ev*math.sin(self.eh), 0.0)
        e.heading = math.degrees(self.eh); e.gear = 4; e.ctrl_mode = 3
        self.pub_ego.publish(e)
        # NPC
        ol = ObjectStatusList(); ol.header.stamp = now; ol.header.frame_id = "map"
        for i, npc in enumerate(self.npcs):
            x, y, h = self._route_pose(npc["s"])
            o = ObjectStatus(); o.unique_id = i; o.type = 1; o.name = "npc%d" % i
            o.heading = math.degrees(h)
            o.position = Vector3(x, y, 0.0)
            o.velocity = Vector3(self.npc_v*math.cos(h), self.npc_v*math.sin(h), 0.0)
            o.size = Vector3(4.5, 2.0, 1.5)
            ol.npc_list.append(o)
        ol.num_of_npcs = len(ol.npc_list)
        self.pub_obj.publish(ol)
        # TF map→base_link
        t = TransformStamped()
        t.header.stamp = now; t.header.frame_id = "map"; t.child_frame_id = "base_link"
        t.transform.translation = Vector3(self.ex, self.ey, 0.0)
        t.transform.rotation = yaw_quat(self.eh)
        self.tfb.sendTransform(t)
        # 동적 마커(ego + NPC)
        self._pub_dynamic_markers(now)

    # ── 마커 ──
    def _line_marker(self, ns, mid, color, width, frame="map"):
        m = Marker(); m.header.frame_id = frame; m.ns = ns; m.id = mid
        m.type = Marker.LINE_LIST; m.action = Marker.ADD
        m.scale.x = width; m.color = color; m.pose.orientation.w = 1.0
        return m

    def _pub_static_markers(self):
        arr = MarkerArray()
        lanes = self._line_marker("lanes", 0, ColorRGBA(0.45, 0.52, 0.60, 0.55), 0.18)
        otz   = self._line_marker("ot_zone", 1, ColorRGBA(0.25, 0.75, 0.35, 0.95), 0.5)
        for L in self.links.values():
            tgt = otz if (L["can_l"] or L["can_r"]) else lanes
            for a, b in zip(L["pts"], L["pts"][1:]):
                tgt.points.append(Point(a[0], a[1], 0.0)); tgt.points.append(Point(b[0], b[1], 0.0))
        route = Marker(); route.header.frame_id = "map"; route.ns = "route"; route.id = 2
        route.type = Marker.LINE_STRIP; route.action = Marker.ADD; route.scale.x = 0.3
        route.color = ColorRGBA(0.35, 0.55, 0.95, 0.8); route.pose.orientation.w = 1.0
        for x, y in self.route: route.points.append(Point(x, y, 0.05))
        arr.markers = [lanes, otz, route]
        self.pub_mk.publish(arr)

    def _pub_dynamic_markers(self, now):
        arr = MarkerArray()
        eg = Marker(); eg.header.frame_id = "map"; eg.header.stamp = now
        eg.ns = "ego"; eg.id = 0; eg.type = Marker.CUBE; eg.action = Marker.ADD
        eg.pose.position = Point(self.ex, self.ey, 0.75); eg.pose.orientation = yaw_quat(self.eh)
        eg.scale = Vector3(4.5, 2.0, 1.5); eg.color = ColorRGBA(0.30, 0.60, 1.0, 0.95)
        arr.markers.append(eg)
        txt = Marker(); txt.header.frame_id = "map"; txt.header.stamp = now
        txt.ns = "ego_v"; txt.id = 0; txt.type = Marker.TEXT_VIEW_FACING; txt.action = Marker.ADD
        txt.pose.position = Point(self.ex, self.ey, 3.0); txt.scale.z = 1.6
        txt.color = ColorRGBA(1, 1, 1, 0.95); txt.text = "%.0f km/h" % (self.ev * 3.6)
        arr.markers.append(txt)
        for i, npc in enumerate(self.npcs):
            x, y, h = self._route_pose(npc["s"])
            nm = Marker(); nm.header.frame_id = "map"; nm.header.stamp = now
            nm.ns = "npc"; nm.id = i; nm.type = Marker.CUBE; nm.action = Marker.ADD
            nm.pose.position = Point(x, y, 0.75); nm.pose.orientation = yaw_quat(h)
            nm.scale = Vector3(4.5, 2.0, 1.5); nm.color = ColorRGBA(1.0, 0.40, 0.30, 0.95)
            arr.markers.append(nm)
        self.pub_mk.publish(arr)

    def spin(self):
        r = rospy.Rate(1.0 / self.dt)
        while not rospy.is_shutdown():
            self._drive(); self._publish()
            r.sleep()


if __name__ == "__main__":
    try: PracticeSim().spin()
    except rospy.ROSInterruptException: pass
