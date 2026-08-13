#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lightweight HD map lane-change planner.

Input:
  - /localization/ego_status
  - /Object_topic
  - /lane_change_cmd (std_msgs/Int32): target lane number. 0 clears the target.

Output:
  - /avoid_waypoints: path_follower_node compatible waypoints
  - /avoid_target_vel: target speed in m/s
  - /avoid_path: RViz/Foxglove path
  - /lane_change_status: short text status
"""
import json
import math
import zipfile
from collections import defaultdict

import rospy
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Int32, String

MAX_HEADING_DIFF = math.radians(70)
FALLBACK_HEADING_DIFF = math.radians(85)


def norm(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Link:
    __slots__ = (
        "idx", "fr", "to", "pts", "lane", "road",
        "can_l", "can_r", "dst_l", "dst_r", "length", "vmax",
    )

    def __init__(self, data):
        self.idx = data["idx"]
        self.fr = data["from_node_idx"]
        self.to = data["to_node_idx"]
        self.pts = [(p[0], p[1]) for p in data["points"]]
        self.lane = data.get("ego_lane")
        self.road = data.get("road_id")
        self.can_l = bool(data.get("can_move_left_lane"))
        self.can_r = bool(data.get("can_move_right_lane"))
        self.dst_l = data.get("left_lane_change_dst_link_idx")
        self.dst_r = data.get("right_lane_change_dst_link_idx")
        self.vmax = (data.get("max_speed") or 50) / 3.6
        self.length = data.get("link_length") or sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(self.pts, self.pts[1:])
        )


class HdmapLaneChange:
    def __init__(self):
        rospy.init_node("hdmap_lane_change")
        hdmap_zip = rospy.get_param(
            "~hdmap_zip", "/home/david/morai-mpc-agent-morai-lio-gps-integration/src/moraimpc/data/hdmap.zip"
        )
        self.cruise_mps = float(rospy.get_param("~cruise_mps", 9.0))
        self.route_len = float(rospy.get_param("~route_len_m", 180.0))
        self.transition_m = float(rospy.get_param("~lane_change_trans_m", 28.0))
        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))
        self.arrive_lat_m = float(rospy.get_param("~arrive_lat_m", 0.8))
        self.arrive_lane_lat_m = float(rospy.get_param("~arrive_lane_lat_m", max(self.arrive_lat_m, 1.5)))
        self.arrive_forced_lane_lat_m = float(rospy.get_param("~arrive_forced_lane_lat_m", self.arrive_lat_m))
        self.path_file = rospy.get_param("~path_file", "")
        self.use_base_route = bool(rospy.get_param("~use_base_route", True))
        self.loop_hdmap_route = bool(rospy.get_param("~loop_hdmap_route", True))
        self.max_forward_links = int(rospy.get_param("~max_forward_links", 500))
        self.anchor_current_pose = bool(rospy.get_param("~anchor_current_pose", False))
        self.auto_avoid = bool(rospy.get_param("~auto_avoid", True))
        self.detect_dist_m = float(rospy.get_param("~detect_dist_m", 45.0))
        self.block_lat_m = float(rospy.get_param("~block_lat_m", 1.7))
        self.target_front_gap_m = float(rospy.get_param("~target_front_gap_m", 18.0))
        self.target_rear_gap_m = float(rospy.get_param("~target_rear_gap_m", 12.0))
        self.gap_front_min_m = float(rospy.get_param("~gap_front_min_m", 7.0))
        self.gap_rear_min_m = float(rospy.get_param("~gap_rear_min_m", 5.0))
        self.gap_front_time_s = float(rospy.get_param("~gap_front_time_s", 0.55))
        self.gap_rear_time_s = float(rospy.get_param("~gap_rear_time_s", 0.45))
        self.gap_rel_speed_time_s = float(rospy.get_param("~gap_rel_speed_time_s", 0.7))
        self.gap_predict_time_s = float(rospy.get_param("~gap_predict_time_s", 1.4))
        self.clear_cycles_needed = int(rospy.get_param("~clear_cycles_needed", 8))
        self.brake_decel = float(rospy.get_param("~brake_decel", 5.0))
        self.front_safety_gap = float(rospy.get_param("~front_safety_gap_m", 7.0))
        self.front_safety_lat_m = float(rospy.get_param("~front_safety_lat_m", 3.0))
        self.object_heading_diff = float(rospy.get_param("~object_heading_diff_deg", 100.0))
        self.require_rejoin_path = bool(rospy.get_param("~require_rejoin_path", True))
        self.rejoin_check_m = float(rospy.get_param("~rejoin_check_m", 140.0))
        self.urgent_transition_min_m = float(rospy.get_param("~urgent_transition_min_m", 10.0))
        self.urgent_transition_margin_m = float(rospy.get_param("~urgent_transition_margin_m", 4.0))
        self.avoid_change_min_mps = float(rospy.get_param("~avoid_change_min_kmh", 6.0)) / 3.6
        self.avoid_hard_stop_m = float(rospy.get_param("~avoid_hard_stop_m", 4.0))
        self.max_avoid_hops = int(rospy.get_param("~max_avoid_hops", 4))

        self.links = {}
        self.adj = defaultdict(list)
        self._load_map(hdmap_zip)
        self.base_route = self._load_base_route(self.path_file)

        self.ego = None
        self.ego_v = 0.0
        self.objects = []
        self.ref_link = None
        self.last_link = None
        self.target_lane = int(rospy.get_param("~target_lane", 0))
        self.active_target_lane = None
        self.change_dst_idx = None
        self.changing = False
        self.mode = "CRUISE"
        self.home_lane = None
        self.clear_cycles = 0
        self.auto_speed_cap = None
        self.change_blend_m = self.transition_m
        self.avoid_hops = 0

        rospy.Subscriber("/localization/ego_status",
                         EgoVehicleStatus, self._ego_cb, queue_size=1)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self._obj_cb, queue_size=1)
        rospy.Subscriber("/lane_change_cmd", Int32, self._cmd_cb, queue_size=1)
        self.pub_wps = rospy.Publisher("/avoid_waypoints", String, queue_size=1)
        self.pub_vel = rospy.Publisher("/avoid_target_vel", Float32, queue_size=1)
        self.pub_path = rospy.Publisher("/avoid_path", Path, queue_size=1)
        self.pub_status = rospy.Publisher("/lane_change_status", String, queue_size=1)

        rospy.loginfo("[LaneChange] ON - link=%d route_wp=%d base_route=%s hdmap_loop=%s ego_anchor=%s auto_avoid=%s target_lane=%d",
                      len(self.links), len(self.base_route),
                      "ON" if self.use_base_route else "OFF",
                      "ON" if self.loop_hdmap_route else "OFF",
                      "ON" if self.anchor_current_pose else "OFF",
                      "ON" if self.auto_avoid else "OFF", self.target_lane)

    def _load_map(self, hdmap_zip):
        with zipfile.ZipFile(hdmap_zip) as zf:
            raw_links = json.loads(zf.read("link_set.json"))
        for data in raw_links:
            link = Link(data)
            if len(link.pts) < 2 or link.lane is None:
                continue
            self.links[link.idx] = link
            self.adj[link.fr].append(link)

    @staticmethod
    def _load_base_route(path_file):
        if not path_file:
            return []
        try:
            with open(path_file) as f:
                data = json.load(f)
            out = []
            for w in data.get("waypoints", []):
                if "x" in w and "y" in w:
                    out.append((float(w["x"]), float(w["y"])))
            return out
        except Exception as exc:
            rospy.logwarn("[LaneChange] base route load failed: %s", exc)
            return []

    def _ego_cb(self, msg):
        self.ego = msg
        self.ego_v = math.hypot(msg.velocity.x, msg.velocity.y)

    def _obj_cb(self, msg):
        objects = []
        for group in (msg.npc_list, msg.pedestrian_list, msg.obstacle_list):
            for obj in group:
                objects.append((obj.position.x, obj.position.y, obj.velocity.x, obj.velocity.y))
        self.objects = objects

    def _cmd_cb(self, msg):
        self.target_lane = int(msg.data)
        if self.target_lane <= 0:
            self.active_target_lane = None
            self.change_dst_idx = None
            self.changing = False
            self.mode = "CRUISE"
            self.home_lane = None
            rospy.loginfo("[LaneChange] 차선변경 목표 해제")
        else:
            rospy.loginfo("[LaneChange] 목표 차선: %d차선", self.target_lane)

    @staticmethod
    def _proj(px, py, link):
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

    def _current_link(self, ex, ey, eh, want_lane=None):
        best = None
        best_score = 1e18
        fallback = None
        fallback_dist = 1e18
        for link in self.links.values():
            if want_lane is not None and link.lane != want_lane:
                continue
            lat, dist, th = self._proj(ex, ey, link)
            if dist > 8.0:
                continue
            hd = abs(norm(th - eh))
            if want_lane is None and hd < FALLBACK_HEADING_DIFF and dist < fallback_dist:
                fallback = link
                fallback_dist = dist
            if hd > MAX_HEADING_DIFF:
                continue
            score = abs(lat) + 0.4 * hd
            if want_lane is None and self.last_link == link.idx:
                score -= 0.7
            if score < best_score:
                best_score = score
                best = link
        if best is None:
            best = fallback
        if best is not None and want_lane is None:
            self.last_link = best.idx
        return best

    def _lat_to(self, ex, ey, link):
        lat, _, _ = self._proj(ex, ey, link)
        return abs(lat)

    def _target_lane_reached(self, cur, ex, ey, eh):
        if self.active_target_lane is None:
            return None
        if cur is not None and cur.lane == self.active_target_lane and self._lat_to(ex, ey, cur) < self.arrive_lane_lat_m:
            return cur
        detected = self._current_link(ex, ey, eh, want_lane=self.active_target_lane)
        if detected is not None and detected.lane == self.active_target_lane and self._lat_to(ex, ey, detected) < self.arrive_forced_lane_lat_m:
            return detected
        return None

    def _forward(self, start, length):
        chain = [start]
        total = start.length
        used = {start.idx}
        cur = start
        while total < length and len(chain) < self.max_forward_links:
            nxt = list(self.adj.get(cur.to, []))
            if not self.loop_hdmap_route:
                nxt = [link for link in nxt if link.idx not in used]
            if not nxt:
                break
            same_lane = [link for link in nxt if link.lane == cur.lane]
            same_road = [link for link in nxt if link.road == cur.road]
            pool = same_lane or same_road or nxt
            cur_th = math.atan2(cur.pts[-1][1] - cur.pts[-2][1], cur.pts[-1][0] - cur.pts[-2][0])
            best = min(
                pool,
                key=lambda link: abs(norm(
                    math.atan2(link.pts[1][1] - link.pts[0][1], link.pts[1][0] - link.pts[0][0]) - cur_th
                )),
            )
            best_th = math.atan2(best.pts[1][1] - best.pts[0][1], best.pts[1][0] - best.pts[0][0])
            if abs(norm(best_th - cur_th)) > math.radians(55):
                break
            chain.append(best)
            used.add(best.idx)
            total += best.length
            cur = best
        return chain

    @staticmethod
    def _chain_pts_raw(chain):
        pts = []
        for link in chain:
            for point in link.pts:
                if pts and (pts[-1][0] - point[0]) ** 2 + (pts[-1][1] - point[1]) ** 2 < 0.04:
                    continue
                pts.append(point)
        return pts

    @staticmethod
    def _resample(pts, step=0.5):
        if len(pts) < 2:
            return list(pts)
        out = [pts[0]]
        carry = 0.0
        for i in range(1, len(pts)):
            ax, ay = out[-1]
            bx, by = pts[i]
            dist = math.hypot(bx - ax, by - ay)
            if dist < 1e-9:
                continue
            sx, sy = ax, ay
            while carry + dist >= step:
                t = (step - carry) / dist
                nx = sx + (bx - sx) * t
                ny = sy + (by - sy) * t
                out.append((nx, ny))
                sx, sy = nx, ny
                dist = math.hypot(bx - sx, by - sy)
                carry = 0.0
            carry += dist
        out.append(pts[-1])
        return out

    def _route_to_link(self, ex, ey, link, blend_m=None, force_anchor=False):
        ref = self._resample(self._chain_pts_raw(self._forward(link, self.route_len)), 0.5)
        if len(ref) < 2:
            return ref
        closest = min(range(len(ref)), key=lambda i: (ref[i][0] - ex) ** 2 + (ref[i][1] - ey) ** 2)
        next_i = min(closest + 1, len(ref) - 1)
        th0 = math.atan2(ref[next_i][1] - ref[closest][1], ref[next_i][0] - ref[closest][0])
        d0 = -math.sin(th0) * (ex - ref[closest][0]) + math.cos(th0) * (ey - ref[closest][1])
        blend = max(1.0, blend_m if blend_m is not None else self.transition_m)
        route = []
        if force_anchor or self.anchor_current_pose:
            route.append((ex, ey))
        travelled = 0.0
        prev = ref[closest]
        for i in range(closest, len(ref)):
            if i > closest:
                travelled += math.hypot(ref[i][0] - prev[0], ref[i][1] - prev[1])
            prev = ref[i]
            u = min(1.0, travelled / blend)
            # Smoothstep gives zero slope at the target centerline and reduces steering chatter.
            w = u * u * (3.0 - 2.0 * u)
            off = d0 * (1.0 - w)
            j = min(i + 1, len(ref) - 1)
            th = math.atan2(ref[j][1] - ref[i][1], ref[j][0] - ref[i][0])
            point = (ref[i][0] - math.sin(th) * off, ref[i][1] + math.cos(th) * off)
            if not route or (route[-1][0] - point[0]) ** 2 + (route[-1][1] - point[1]) ** 2 > 0.04:
                route.append(point)
        return route

    def _base_route_slice(self, ex, ey):
        if len(self.base_route) < 2:
            return []
        closest = min(
            range(len(self.base_route)),
            key=lambda i: (self.base_route[i][0] - ex) ** 2 + (self.base_route[i][1] - ey) ** 2,
        )
        pts = self.base_route[closest:]
        if not pts:
            return []
        out = []
        total = 0.0
        prev = pts[0]
        for point in pts:
            total += math.hypot(point[0] - prev[0], point[1] - prev[1])
            out.append(point)
            prev = point
            if total >= self.route_len:
                break
        return out

    def _link_reference(self, link):
        ref = self._resample(self._chain_pts_raw(self._forward(link, self.route_len)), 0.5)
        if len(ref) < 2:
            return [], [], []
        s = [0.0]
        th = []
        for i in range(1, len(ref)):
            s.append(s[-1] + math.hypot(ref[i][0] - ref[i - 1][0], ref[i][1] - ref[i - 1][1]))
        for i in range(len(ref)):
            a = max(0, i - 1)
            b = min(len(ref) - 1, i + 1)
            th.append(math.atan2(ref[b][1] - ref[a][1], ref[b][0] - ref[a][0]))
        return ref, s, th

    def _object_matches_link_dir(self, th, vx, vy):
        speed = math.hypot(vx, vy)
        if speed < 0.5:
            return True
        obj_h = math.atan2(vy, vx)
        return abs(norm(obj_h - th)) <= math.radians(self.object_heading_diff)

    def _object_lead_on_link(self, link, ex, ey, min_s=2.0, max_s=None):
        max_s = self.detect_dist_m if max_s is None else max_s
        ref, s, th = self._link_reference(link)
        if len(ref) < 2:
            return None
        ego_i = min(range(len(ref)), key=lambda i: (ref[i][0] - ex) ** 2 + (ref[i][1] - ey) ** 2)
        best = None
        for ox, oy, vx, vy in self.objects:
            obj_i = min(range(len(ref)), key=lambda i: (ref[i][0] - ox) ** 2 + (ref[i][1] - oy) ** 2)
            s_rel = s[obj_i] - s[ego_i]
            if s_rel < min_s or s_rel > max_s:
                continue
            lat = -math.sin(th[obj_i]) * (ox - ref[obj_i][0]) + math.cos(th[obj_i]) * (oy - ref[obj_i][1])
            if abs(lat) > self.block_lat_m:
                continue
            if not self._object_matches_link_dir(th[obj_i], vx, vy):
                continue
            v_along = vx * math.cos(th[obj_i]) + vy * math.sin(th[obj_i])
            item = (s_rel, lat, v_along, ox, oy)
            if best is None or s_rel < best[0]:
                best = item
        return best

    def _target_lane_clear(self, link, ex, ey):
        ok, _reason, _score = self._target_lane_gap_ok(link, ex, ey)
        return ok

    def _target_lane_gap_ok(self, link, ex, ey):
        ref, s, th = self._link_reference(link)
        if len(ref) < 2:
            return False, "no-reference", -1e9
        ego_i = min(range(len(ref)), key=lambda i: (ref[i][0] - ex) ** 2 + (ref[i][1] - ey) ** 2)
        front = None
        rear = None
        for ox, oy, vx, vy in self.objects:
            obj_i = min(range(len(ref)), key=lambda i: (ref[i][0] - ox) ** 2 + (ref[i][1] - oy) ** 2)
            s_rel = s[obj_i] - s[ego_i]
            lat = -math.sin(th[obj_i]) * (ox - ref[obj_i][0]) + math.cos(th[obj_i]) * (oy - ref[obj_i][1])
            if abs(lat) >= self.block_lat_m + 0.4:
                continue
            if not self._object_matches_link_dir(th[obj_i], vx, vy):
                continue
            v_along = vx * math.cos(th[obj_i]) + vy * math.sin(th[obj_i])
            item = (s_rel, v_along)
            if s_rel >= 0.0:
                if front is None or s_rel < front[0]:
                    front = item
            else:
                if rear is None or s_rel > rear[0]:
                    rear = item

        ego_v = max(0.0, self.ego_v)
        front_req = self.gap_front_min_m
        rear_req = self.gap_rear_min_m
        front_gap = self.target_front_gap_m
        rear_gap = self.target_rear_gap_m
        if front is not None:
            front_gap = front[0]
            closing = max(0.0, ego_v - front[1])
            front_req = max(front_req, ego_v * self.gap_front_time_s + closing * self.gap_rel_speed_time_s)
        if rear is not None:
            rear_gap = -rear[0]
            closing = max(0.0, rear[1] - ego_v)
            rear_req = max(rear_req, rear[1] * self.gap_rear_time_s + closing * self.gap_rel_speed_time_s)

        front_pred = front_gap
        rear_pred = rear_gap
        if front is not None:
            front_pred = front[0] + (front[1] - ego_v) * self.gap_predict_time_s
        if rear is not None:
            rear_pred = -rear[0] + (ego_v - rear[1]) * self.gap_predict_time_s

        if front_gap < front_req:
            return False, "front-gap %.1f<%.1f" % (front_gap, front_req), front_gap - front_req
        if rear_gap < rear_req:
            return False, "rear-gap %.1f<%.1f" % (rear_gap, rear_req), rear_gap - rear_req
        if front_pred < self.gap_front_min_m:
            return False, "front-pred %.1f<%.1f" % (front_pred, self.gap_front_min_m), front_pred - self.gap_front_min_m
        if rear_pred < self.gap_rear_min_m:
            return False, "rear-pred %.1f<%.1f" % (rear_pred, self.gap_rear_min_m), rear_pred - self.gap_rear_min_m
        score = min(
            front_gap - front_req,
            rear_gap - rear_req,
            front_pred - self.gap_front_min_m,
            rear_pred - self.gap_rear_min_m,
        )
        return True, (
            "gap front=%.1f/%.1f pred=%.1f rear=%.1f/%.1f pred=%.1f"
            % (front_gap, front_req, front_pred, rear_gap, rear_req, rear_pred)
        ), score

    def _link_front_speed_cap(self, link, ex, ey, safety_gap=None):
        safety_gap = self.front_safety_gap if safety_gap is None else safety_gap
        ref, s, th = self._link_reference(link)
        if len(ref) < 2:
            return None
        ego_i = min(range(len(ref)), key=lambda i: (ref[i][0] - ex) ** 2 + (ref[i][1] - ey) ** 2)
        cap = None
        for ox, oy, vx, vy in self.objects:
            obj_i = min(range(len(ref)), key=lambda i: (ref[i][0] - ox) ** 2 + (ref[i][1] - oy) ** 2)
            s_rel = s[obj_i] - s[ego_i]
            if s_rel <= 0.0 or s_rel > self.detect_dist_m:
                continue
            lat = -math.sin(th[obj_i]) * (ox - ref[obj_i][0]) + math.cos(th[obj_i]) * (oy - ref[obj_i][1])
            if abs(lat) > self.front_safety_lat_m:
                continue
            if not self._object_matches_link_dir(th[obj_i], vx, vy):
                continue
            obj_v = vx * math.cos(th[obj_i]) + vy * math.sin(th[obj_i])
            stop_space = max(0.0, s_rel - safety_gap)
            safe_v = math.sqrt(max(0.0, 2.0 * self.brake_decel * stop_space)) + max(0.0, obj_v)
            cap = safe_v if cap is None else min(cap, safe_v)
        return cap

    def _front_object_speed_cap(self, ex, ey, eh):
        cap = None
        ce = math.cos(eh)
        se = math.sin(eh)
        for ox, oy, vx, vy in self.objects:
            dx = ox - ex
            dy = oy - ey
            s_rel = ce * dx + se * dy
            lat = -se * dx + ce * dy
            if s_rel <= 0.0 or s_rel > self.detect_dist_m:
                continue
            if abs(lat) > self.front_safety_lat_m:
                continue
            obj_v = vx * ce + vy * se
            stop_space = max(0.0, s_rel - self.front_safety_gap)
            safe_v = math.sqrt(max(0.0, 2.0 * self.brake_decel * stop_space)) + max(0.0, obj_v)
            cap = safe_v if cap is None else min(cap, safe_v)
        return cap

    def _has_rejoin_path(self, cur, cand, min_return_s):
        if not self.require_rejoin_path:
            return True, "rejoin-check-off"
        home_chain = self._forward(cur, self.rejoin_check_m + min_return_s)
        home_ids = {link.idx for link in home_chain}
        cand_chain = self._forward(cand, self.rejoin_check_m + min_return_s)
        travelled = 0.0
        for link in cand_chain:
            link_end_s = travelled + link.length
            if link_end_s >= min_return_s:
                for side, adj in self._adjacent_candidates(link):
                    if adj.idx in home_ids:
                        return True, "rejoin-%s:%s->%s" % (side, link.idx, adj.idx)
            travelled = link_end_s
        return False, "no-return-path"

    def _choose_avoid_link(self, cur, ex, ey, blocker_s=None):
        candidates = self._adjacent_candidates(cur)
        if not candidates:
            return None, "no-adjacent-lane"
        clear = []
        gap_reasons = []
        for side, link in candidates:
            ok, gap_reason, gap_score = self._target_lane_gap_ok(link, ex, ey)
            if ok:
                clear.append((side, link, gap_reason, gap_score))
            else:
                gap_reasons.append("%s:%s" % (side, gap_reason))
        if not clear:
            return None, "target-blocked " + ",".join(gap_reasons)
        min_return_s = max(self.transition_m, (blocker_s or 0.0) + self.front_safety_gap)
        rejoinable = []
        blocked_reasons = []
        for side, link, gap_reason, gap_score in clear:
            ok, reason = self._has_rejoin_path(cur, link, min_return_s)
            if ok:
                rejoinable.append((side, link, reason, gap_reason, gap_score))
            else:
                blocked_reasons.append("%s:%s" % (side, reason))
        if not rejoinable:
            return None, "no-return-path"
        # Prefer the adjacent lane with the largest usable gap; this lets traffic merges happen promptly.
        if len(rejoinable) == 1:
            side, link, reason, gap_reason, _gap_score = rejoinable[0]
            return link, "%s/%s/%s" % (side, reason, gap_reason)
        side, link, reason, gap_reason, _gap_score = max(
            rejoinable,
            key=lambda item: (item[4], -abs(item[1].lane - cur.lane)),
        )
        return link, "%s/%s/%s" % (side, reason, gap_reason)

    def _avoid_blend_m(self, blocker_s=None):
        if blocker_s is None:
            return self.transition_m
        urgent = max(self.urgent_transition_min_m, blocker_s - self.urgent_transition_margin_m)
        return min(self.transition_m, urgent)

    def _allow_avoid_crawl(self, blocker):
        if blocker is None or blocker[0] <= self.avoid_hard_stop_m:
            return
        if self.auto_speed_cap is not None:
            self.auto_speed_cap = max(self.auto_speed_cap, self.avoid_change_min_mps)

    def _prefer_target_lane_speed(self, link, ex, ey):
        target_cap = self._link_front_speed_cap(link, ex, ey)
        if target_cap is None:
            self.auto_speed_cap = None
        else:
            self.auto_speed_cap = target_cap

    def _lane_change_route(self, cur, dst, ex, ey):
        cur_pts = self._route_to_link(ex, ey, cur)
        dst_pts = self._resample(self._chain_pts_raw(self._forward(dst, self.route_len)), 0.5)
        if len(cur_pts) < 2 or len(dst_pts) < 2:
            return cur_pts
        trans_n = max(4, int(self.transition_m / 0.5))
        route = []
        for i, point in enumerate(cur_pts[:trans_n]):
            w = i / max(1, trans_n - 1)
            j = min(range(len(dst_pts)), key=lambda k: (dst_pts[k][0] - point[0]) ** 2 + (dst_pts[k][1] - point[1]) ** 2)
            route.append(((1.0 - w) * point[0] + w * dst_pts[j][0], (1.0 - w) * point[1] + w * dst_pts[j][1]))
        if route:
            j = min(range(len(dst_pts)), key=lambda k: (dst_pts[k][0] - route[-1][0]) ** 2 + (dst_pts[k][1] - route[-1][1]) ** 2)
            route.extend(dst_pts[j + 1:])
        return route

    def _adjacent_candidates(self, cur):
        out = []
        if cur.can_l and cur.dst_l in self.links:
            out.append(("left", self.links[cur.dst_l]))
        if cur.can_r and cur.dst_r in self.links:
            out.append(("right", self.links[cur.dst_r]))
        return out

    def _select_next_lane_link(self, cur, target_lane):
        candidates = self._adjacent_candidates(cur)
        if not candidates:
            return None, "no-adjacent"
        exact = [(side, link) for side, link in candidates if link.lane == target_lane]
        if exact:
            return exact[0][1], exact[0][0]
        side, link = min(candidates, key=lambda item: abs(item[1].lane - target_lane))
        if abs(link.lane - target_lane) < abs(cur.lane - target_lane):
            return link, side
        return None, "no-progress"

    def _auto_route(self, cur, ex, ey, eh):
        self.auto_speed_cap = self._front_object_speed_cap(ex, ey, eh)
        blocker = self._object_lead_on_link(cur, ex, ey)
        if blocker is not None:
            follow_cap = max(0.0, blocker[2] + 0.12 * max(0.0, blocker[0] - self.front_safety_gap))
            self.auto_speed_cap = follow_cap if self.auto_speed_cap is None else min(self.auto_speed_cap, follow_cap)

        if self.mode == "CRUISE":
            self.home_lane = cur.lane
            if blocker is not None:
                dst, reason = self._choose_avoid_link(cur, ex, ey, blocker[0])
                if dst is not None:
                    self.mode = "AVOID"
                    self.changing = True
                    self.active_target_lane = dst.lane
                    self.change_dst_idx = dst.idx
                    self.change_blend_m = self._avoid_blend_m(blocker[0])
                    self.avoid_hops = 1
                    self._prefer_target_lane_speed(dst, ex, ey)
                    self._allow_avoid_crawl(blocker)
                    self.clear_cycles = 0
                    rospy.loginfo(
                        "[LaneChange] AUTO avoid: lane %d -> %d (%s), object %.1fm ahead, blend %.1fm",
                        cur.lane, dst.lane, reason, blocker[0], self.change_blend_m,
                    )
                    return self._route_to_link(ex, ey, dst, self.change_blend_m), (
                        "AUTO_AVOID road=%s lane=%s target=%s object_s=%.1f blend=%.1f"
                        % (cur.road, cur.lane, dst.lane, blocker[0], self.change_blend_m)
                    )
                return self._route_to_link(ex, ey, cur), (
                    "AUTO_WAIT road=%s lane=%s reason=%s object_s=%.1f"
                    % (cur.road, cur.lane, reason, blocker[0])
                )
            self.avoid_hops = 0
            base = self._base_route_slice(ex, ey) if self.use_base_route else []
            return (base if base else self._route_to_link(ex, ey, cur)), (
                "AUTO_CRUISE road=%s lane=%s link=%s" % (cur.road, cur.lane, cur.idx)
            )

        if self.mode == "AVOID":
            if self.changing and self.change_dst_idx in self.links:
                dst = self.links[self.change_dst_idx]
                detected = self._target_lane_reached(cur, ex, ey, eh)
                if detected is not None:
                    self.changing = False
                    cur = detected
                    rospy.loginfo("[LaneChange] AUTO avoid lane reached: %d", detected.lane)
                else:
                    if blocker is not None:
                        self.change_blend_m = min(self.change_blend_m, self._avoid_blend_m(blocker[0]))
                    if self._target_lane_clear(dst, ex, ey):
                        self._prefer_target_lane_speed(dst, ex, ey)
                        self._allow_avoid_crawl(blocker)
                    object_s = blocker[0] if blocker is not None else -1.0
                    return self._route_to_link(ex, ey, dst, self.change_blend_m), (
                        "AUTO_AVOID_CHANGE road=%s lane=%s target=%s object_s=%.1f blend=%.1f"
                        % (cur.road, cur.lane, dst.lane, object_s, self.change_blend_m)
                    )

            if blocker is not None and self.avoid_hops < self.max_avoid_hops:
                dst, reason = self._choose_avoid_link(cur, ex, ey, blocker[0])
                if dst is not None:
                    self.changing = True
                    self.active_target_lane = dst.lane
                    self.change_dst_idx = dst.idx
                    self.change_blend_m = self._avoid_blend_m(blocker[0])
                    self.avoid_hops += 1
                    self._prefer_target_lane_speed(dst, ex, ey)
                    self._allow_avoid_crawl(blocker)
                    self.clear_cycles = 0
                    rospy.loginfo(
                        "[LaneChange] AUTO chained avoid: lane %d -> %d (%s), object %.1fm ahead, blend %.1fm",
                        cur.lane, dst.lane, reason, blocker[0], self.change_blend_m,
                    )
                    return self._route_to_link(ex, ey, dst, self.change_blend_m), (
                        "AUTO_AVOID_CHAIN road=%s lane=%s target=%s object_s=%.1f blend=%.1f"
                        % (cur.road, cur.lane, dst.lane, blocker[0], self.change_blend_m)
                    )

            home = self._current_link(ex, ey, eh, want_lane=self.home_lane) if self.home_lane is not None else None
            home_blocker = self._object_lead_on_link(home, ex, ey, min_s=-6.0, max_s=self.detect_dist_m) if home is not None else blocker
            if home_blocker is None:
                self.clear_cycles += 1
            else:
                self.clear_cycles = 0

            if self.clear_cycles >= self.clear_cycles_needed and self.home_lane is not None and cur.lane != self.home_lane:
                dst, reason = self._select_next_lane_link(cur, self.home_lane)
                if dst is not None and self._target_lane_clear(dst, ex, ey):
                    self.mode = "RETURN"
                    self.changing = True
                    self.active_target_lane = dst.lane
                    self.change_dst_idx = dst.idx
                    self._prefer_target_lane_speed(dst, ex, ey)
                    rospy.loginfo("[LaneChange] AUTO return: lane %d -> %d", cur.lane, dst.lane)
                    return self._route_to_link(ex, ey, dst, self.transition_m), (
                        "AUTO_RETURN road=%s lane=%s target=%s" % (cur.road, cur.lane, dst.lane)
                    )
                self._prefer_target_lane_speed(cur, ex, ey)
                return self._route_to_link(ex, ey, cur), (
                    "AUTO_RETURN_WAIT road=%s lane=%s reason=%s" % (cur.road, cur.lane, reason)
                )
            self._prefer_target_lane_speed(cur, ex, ey)
            return self._route_to_link(ex, ey, cur), (
                "AUTO_AVOID_KEEP road=%s lane=%s clear=%d" % (cur.road, cur.lane, self.clear_cycles)
            )

        if self.mode == "RETURN":
            if self.changing and self.change_dst_idx in self.links:
                dst = self.links[self.change_dst_idx]
                detected = self._target_lane_reached(cur, ex, ey, eh)
                if detected is not None:
                    self.changing = False
                    cur = detected
                    if detected.lane == self.home_lane:
                        self.mode = "CRUISE"
                        self.home_lane = None
                        self.clear_cycles = 0
                        self.change_blend_m = self.transition_m
                        self.avoid_hops = 0
                        rospy.loginfo("[LaneChange] AUTO route lane restored")
                else:
                    target_clear, target_reason, _target_score = self._target_lane_gap_ok(dst, ex, ey)
                    object_s = blocker[0] if blocker is not None else -1.0
                    if target_clear:
                        self._prefer_target_lane_speed(dst, ex, ey)
                        self._allow_avoid_crawl(blocker)
                        return self._route_to_link(ex, ey, dst, self.transition_m), (
                            "AUTO_RETURN_CHANGE road=%s lane=%s target=%s object_s=%.1f"
                            % (cur.road, cur.lane, dst.lane, object_s)
                        )

                    if blocker is not None:
                        alt, avoid_reason = self._choose_avoid_link(cur, ex, ey, blocker[0])
                        if alt is not None:
                            self.mode = "AVOID"
                            self.changing = True
                            self.active_target_lane = alt.lane
                            self.change_dst_idx = alt.idx
                            self.change_blend_m = self._avoid_blend_m(blocker[0])
                            self.avoid_hops = 1
                            self.clear_cycles = 0
                            self._prefer_target_lane_speed(alt, ex, ey)
                            self._allow_avoid_crawl(blocker)
                            rospy.loginfo(
                                "[LaneChange] AUTO return interrupted: lane %d -> %d (%s), object %.1fm ahead, return target blocked: %s",
                                cur.lane, alt.lane, avoid_reason, blocker[0], target_reason,
                            )
                            return self._route_to_link(ex, ey, alt, self.change_blend_m), (
                                "AUTO_RETURN_DETOUR_CHANGE road=%s lane=%s target=%s object_s=%.1f blend=%.1f reason=%s"
                                % (cur.road, cur.lane, alt.lane, blocker[0], self.change_blend_m, target_reason)
                            )

                    self.mode = "AVOID"
                    self.changing = False
                    self.active_target_lane = None
                    self.change_dst_idx = None
                    self.change_blend_m = self.transition_m
                    self.avoid_hops = 0
                    self.clear_cycles = 0
                    self._prefer_target_lane_speed(cur, ex, ey)
                    rospy.loginfo(
                        "[LaneChange] AUTO return paused: lane %d keeps current lane, return target %d blocked: %s",
                        cur.lane, dst.lane, target_reason,
                    )
                    return self._route_to_link(ex, ey, cur), (
                        "AUTO_RETURN_HOLD road=%s lane=%s target=%s reason=%s"
                        % (cur.road, cur.lane, dst.lane, target_reason)
                    )

            if self.home_lane is not None and cur.lane != self.home_lane:
                dst, reason = self._select_next_lane_link(cur, self.home_lane)
                if dst is not None and self._target_lane_clear(dst, ex, ey):
                    self.changing = True
                    self.active_target_lane = dst.lane
                    self.change_dst_idx = dst.idx
                    self._prefer_target_lane_speed(dst, ex, ey)
                    rospy.loginfo("[LaneChange] AUTO return step: lane %d -> %d", cur.lane, dst.lane)
                    return self._route_to_link(ex, ey, dst, self.transition_m), (
                        "AUTO_RETURN road=%s lane=%s target=%s" % (cur.road, cur.lane, dst.lane)
                    )
                if blocker is not None:
                    dst, avoid_reason = self._choose_avoid_link(cur, ex, ey, blocker[0])
                    if dst is not None:
                        self.mode = "AVOID"
                        self.changing = True
                        self.active_target_lane = dst.lane
                        self.change_dst_idx = dst.idx
                        self.change_blend_m = self._avoid_blend_m(blocker[0])
                        self.avoid_hops = 1
                        self.clear_cycles = 0
                        self._prefer_target_lane_speed(dst, ex, ey)
                        self._allow_avoid_crawl(blocker)
                        rospy.loginfo(
                            "[LaneChange] AUTO return detour: lane %d -> %d (%s), object %.1fm ahead, blend %.1fm",
                            cur.lane, dst.lane, avoid_reason, blocker[0], self.change_blend_m,
                        )
                        return self._route_to_link(ex, ey, dst, self.change_blend_m), (
                            "AUTO_RETURN_DETOUR road=%s lane=%s target=%s object_s=%.1f blend=%.1f reason=%s"
                            % (cur.road, cur.lane, dst.lane, blocker[0], self.change_blend_m, reason)
                        )
                self._prefer_target_lane_speed(cur, ex, ey)
                return self._route_to_link(ex, ey, cur), (
                    "AUTO_RETURN_WAIT road=%s lane=%s reason=%s" % (cur.road, cur.lane, reason)
                )
            self.mode = "CRUISE"
            self.change_blend_m = self.transition_m
            self.avoid_hops = 0
            base = self._base_route_slice(ex, ey) if self.use_base_route else []
            return (base if base else self._route_to_link(ex, ey, cur)), (
                "AUTO_CRUISE road=%s lane=%s link=%s" % (cur.road, cur.lane, cur.idx)
            )

        self.mode = "CRUISE"
        return self._route_to_link(ex, ey, cur), "AUTO_RESET lane=%s" % cur.lane

    @staticmethod
    def _smooth(pts):
        pts = list(pts)
        for _ in range(2):
            if len(pts) < 5:
                break
            out = [pts[0], pts[1]]
            for i in range(2, len(pts) - 2):
                out.append((
                    (pts[i - 2][0] + pts[i - 1][0] + pts[i][0] + pts[i + 1][0] + pts[i + 2][0]) / 5.0,
                    (pts[i - 2][1] + pts[i - 1][1] + pts[i][1] + pts[i + 1][1] + pts[i + 2][1]) / 5.0,
                ))
            out.extend([pts[-2], pts[-1]])
            pts = out
        return pts

    def _publish(self, pts, v_cmd, status):
        pts = self._smooth(self._resample(pts, 1.0))
        if len(pts) < 2:
            return
        waypoints = []
        for i, (x, y) in enumerate(pts):
            j = min(i + 1, len(pts) - 1)
            heading = math.atan2(pts[j][1] - y, pts[j][0] - x)
            waypoints.append({"x": x, "y": y, "heading": heading, "gear": "D"})
        self.pub_wps.publish(String(data=json.dumps({"waypoints": waypoints})))
        self.pub_vel.publish(Float32(data=v_cmd))
        self.pub_status.publish(String(data=status))

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for x, y in pts:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)
        self.pub_path.publish(path)

    def _plan(self):
        if self.ego is None:
            return
        ex = self.ego.position.x
        ey = self.ego.position.y
        eh = math.radians(self.ego.heading)
        cur = self._current_link(ex, ey, eh)
        if cur is None:
            status = "NO_LANE cannot_match_current_pose"
            self.pub_status.publish(String(data=status))
            rospy.logwarn_throttle(2.0, "[LaneChange] %s", status)
            return
        self.auto_speed_cap = None

        if self.target_lane <= 0 and self.auto_avoid:
            route, status = self._auto_route(cur, ex, ey, eh)
        elif self.target_lane <= 0:
            self.changing = False
            self.active_target_lane = None
            self.change_dst_idx = None
            self.mode = "CRUISE"
            self.home_lane = None
            self.change_blend_m = self.transition_m
            self.avoid_hops = 0
            route = self._route_to_link(ex, ey, cur)
            status = "CRUISE road=%s lane=%s link=%s" % (cur.road, cur.lane, cur.idx)
        elif self.changing and self.change_dst_idx in self.links:
            self.mode = "MANUAL"
            dst = self.links[self.change_dst_idx]
            route = self._route_to_link(ex, ey, dst, self.transition_m)
            status = "LANE_CHANGE road=%s lane=%s target=%s step=%s link=%s" % (
                cur.road, cur.lane, self.target_lane, dst.lane, cur.idx
            )
            detected = self._target_lane_reached(cur, ex, ey, eh)
            if detected is not None:
                self.changing = False
                self.change_dst_idx = None
                self.ref_link = detected.idx
                rospy.loginfo("[LaneChange] %d차선 도착", detected.lane)
        elif self.target_lane == cur.lane:
            self.mode = "MANUAL"
            self.active_target_lane = None
            self.change_dst_idx = None
            self.change_blend_m = self.transition_m
            self.avoid_hops = 0
            route = self._route_to_link(ex, ey, cur)
            status = "CRUISE road=%s lane=%s link=%s" % (cur.road, cur.lane, cur.idx)
        else:
            self.mode = "MANUAL"
            dst, side = self._select_next_lane_link(cur, self.target_lane)
            if dst is None:
                route = self._route_to_link(ex, ey, cur)
                status = "WAIT road=%s lane=%s target=%s reason=%s link=%s" % (
                    cur.road, cur.lane, self.target_lane, side, cur.idx
                )
            else:
                self.changing = True
                self.active_target_lane = dst.lane
                self.change_dst_idx = dst.idx
                rospy.loginfo("[LaneChange] %d차선 -> %d차선 (%s)", cur.lane, dst.lane, side)
                route = self._route_to_link(ex, ey, dst, self.transition_m)
                status = "LANE_CHANGE road=%s lane=%s target=%s step=%s link=%s" % (
                    cur.road, cur.lane, self.target_lane, dst.lane, cur.idx
                )

        v_cmd = min(self.cruise_mps, cur.vmax)
        if self.auto_speed_cap is not None:
            v_cmd = min(v_cmd, max(0.0, self.auto_speed_cap))
        self._publish(route, v_cmd, status)
        rospy.loginfo_throttle(
            1.0,
            "[LaneChange] %s v=%.1fkm/h",
            status,
            v_cmd * 3.6,
        )

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self._plan()
            rate.sleep()


if __name__ == "__main__":
    try:
        HdmapLaneChange().spin()
    except rospy.ROSInterruptException:
        pass
