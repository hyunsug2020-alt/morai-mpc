#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
avoidance_planner_node — /Object_topic NPC 인지 → Lattice 회피 후보 생성 → best path /avoid_path 발행
참고: henes_ws/jeju/scripts/LatticePlanner.py (5차 다항식 lattice)
"""
import json
import math
import os
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList


class AvoidancePlanner:
    def __init__(self):
        rospy.init_node('avoidance_planner')

        self.path_file = rospy.get_param('~path_file',
            os.path.expanduser('~/morai-mpc/src/moraimpc/data/mixed.json'))
        self.lane_offsets    = rospy.get_param('~lane_offsets',    [-3.0, -1.5, 0.0, 1.5, 3.0])
        self.lookahead_idx   = rospy.get_param('~lookahead_idx',   30)   # 0.3m 간격 → 9m 앞
        self.safe_distance   = rospy.get_param('~safe_distance',   2.5)  # 충돌 임계
        self.detect_distance = rospy.get_param('~detect_distance', 25.0) # NPC 인지 거리
        self.rate_hz         = rospy.get_param('~rate_hz',         10.0)

        self.ref_waypoints = self._load_waypoints(self.path_file)
        rospy.loginfo(f"[avoid] reference loaded: {len(self.ref_waypoints)} pts ({self.path_file})")

        self.ego = None
        self.objs = None

        rospy.Subscriber('/localization/ego_status',
                         EgoVehicleStatus, self._ego_cb, queue_size=1)
        rospy.Subscriber('/Object_topic', ObjectStatusList, self._obj_cb,  queue_size=1)

        self.pub_path   = rospy.Publisher('/avoid_path',          Path,        queue_size=1)
        self.pub_status = rospy.Publisher('/avoidance_status',    String,      queue_size=1)
        self.pub_viz    = rospy.Publisher('/avoidance_viz',       MarkerArray, queue_size=1)
        self.pub_cands  = [rospy.Publisher(f'/avoid_cand_{i}', Path, queue_size=1)
                           for i in range(len(self.lane_offsets))]

        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)
        rospy.loginfo("[avoid] avoidance_planner ready")
        rospy.spin()

    @staticmethod
    def _load_waypoints(path):
        with open(path) as f:
            data = json.load(f)
        wps = data['waypoints'] if isinstance(data, dict) else data
        return [(p['x'], p['y'], p.get('heading', 0.0)) for p in wps]

    def _ego_cb(self, msg):
        self.ego = msg

    def _obj_cb(self, msg):
        self.objs = msg

    def _nearest_idx(self, x, y):
        d2_min, i_min = float('inf'), 0
        for i, (px, py, _) in enumerate(self.ref_waypoints):
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 < d2_min:
                d2_min, i_min = d2, i
        return i_min, math.sqrt(d2_min)

    def _nearby_npcs(self, ego_x, ego_y):
        if self.objs is None:
            return []
        out = []
        for n in self.objs.npc_list:
            d = math.hypot(n.position.x - ego_x, n.position.y - ego_y)
            if d <= self.detect_distance:
                out.append((n.position.x, n.position.y, d, n.unique_id))
        return out

    def _quintic_path(self, start, start_yaw, start_v, end, end_yaw, T):
        sx, sy = start
        ex, ey = end
        svx, svy = start_v * math.cos(start_yaw), start_v * math.sin(start_yaw)
        evx, evy = start_v * math.cos(end_yaw),   start_v * math.sin(end_yaw)
        mat_T = np.array([[T**3, T**4, T**5],
                          [3*T**2, 4*T**3, 5*T**4],
                          [6*T,   12*T**2, 20*T**3]])
        if abs(np.linalg.det(mat_T)) < 1e-9:
            return None
        inv = np.linalg.inv(mat_T)
        ax = inv.dot([ex - sx - svx * T, evx - svx, 0.0])
        ay = inv.dot([ey - sy - svy * T, evy - svy, 0.0])
        cx = [sx, svx, 0.0, ax[0], ax[1], ax[2]]
        cy = [sy, svy, 0.0, ay[0], ay[1], ay[2]]
        N = max(10, int(T * 10))
        pts = []
        for t in np.linspace(0.0, T, N):
            x = sum(cx[i] * t**i for i in range(6))
            y = sum(cy[i] * t**i for i in range(6))
            pts.append((x, y))
        return pts

    def _collision_cost(self, path_pts, npcs):
        cost = 0.0
        for (ox, oy, _d, _id) in npcs:
            min_d = min(math.hypot(px - ox, py - oy) for px, py in path_pts)
            if min_d < self.safe_distance:
                cost += 100.0 * (1.0 - min_d / self.safe_distance)
        return cost

    def _tick(self, _evt):
        if self.ego is None:
            return

        ego_x = self.ego.position.x
        ego_y = self.ego.position.y
        ego_v = max(0.5, abs(self.ego.velocity.x))
        idx, _ = self._nearest_idx(ego_x, ego_y)
        nearby = self._nearby_npcs(ego_x, ego_y)

        if not nearby:
            self.pub_status.publish("NORMAL")
            self._publish_ref_path(idx)
            return

        i_end = min(idx + self.lookahead_idx, len(self.ref_waypoints) - 1)
        ref_x, ref_y, _ = self.ref_waypoints[i_end]
        if i_end > 0:
            prev_x, prev_y, _ = self.ref_waypoints[i_end - 1]
            ref_yaw = math.atan2(ref_y - prev_y, ref_x - prev_x)
        else:
            ref_yaw = self.ref_waypoints[i_end][2]

        cur_yaw = self.ref_waypoints[idx][2]
        T_seg = max(1.0, self.lookahead_idx * 0.3 / ego_v)

        candidates = []
        for k, off in enumerate(self.lane_offsets):
            end_x = ref_x + off * math.cos(ref_yaw + math.pi / 2)
            end_y = ref_y + off * math.sin(ref_yaw + math.pi / 2)
            pts = self._quintic_path((ego_x, ego_y), cur_yaw, ego_v,
                                     (end_x, end_y), ref_yaw, T_seg)
            if pts is None:
                candidates.append((None, float('inf'), off))
                continue
            cost = self._collision_cost(pts, nearby) + abs(off) * 1.0
            candidates.append((pts, cost, off))
            self._publish_path_topic(self.pub_cands[k], pts)

        candidates_valid = [(pts, c, o) for (pts, c, o) in candidates if pts is not None]
        if not candidates_valid:
            self.pub_status.publish("FAIL")
            return

        best = min(candidates_valid, key=lambda t: t[1])
        best_pts, best_cost, best_off = best

        if best_cost >= 100.0:
            self.pub_status.publish("BLOCKED")
            return

        self.pub_status.publish(f"AVOID off={best_off:.2f} cost={best_cost:.1f}")
        self._publish_path_topic(self.pub_path, best_pts)
        self._publish_viz(nearby, best_pts)

    def _publish_ref_path(self, idx):
        i_end = min(idx + self.lookahead_idx, len(self.ref_waypoints) - 1)
        pts = [(p[0], p[1]) for p in self.ref_waypoints[idx:i_end + 1]]
        self._publish_path_topic(self.pub_path, pts)

    @staticmethod
    def _publish_path_topic(pub, pts):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        for x, y in pts:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        pub.publish(msg)

    def _publish_viz(self, nearby, best_pts):
        arr = MarkerArray()
        now = rospy.Time.now()
        for i, (x, y, _d, uid) in enumerate(nearby):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns, m.id = 'npc', i
            m.type, m.action = Marker.CYLINDER, Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = self.safe_distance * 2.0
            m.scale.z = 1.0
            m.color.r, m.color.a = 1.0, 0.4
            arr.markers.append(m)
        line = Marker()
        line.header.frame_id = 'map'
        line.header.stamp = now
        line.ns, line.id = 'best_path', 0
        line.type, line.action = Marker.LINE_STRIP, Marker.ADD
        line.scale.x = 0.25
        line.color.g, line.color.a = 1.0, 1.0
        line.pose.orientation.w = 1.0
        from geometry_msgs.msg import Point
        line.points = [Point(x=x, y=y, z=0.1) for x, y in best_pts]
        arr.markers.append(line)
        self.pub_viz.publish(arr)


if __name__ == '__main__':
    try:
        AvoidancePlanner()
    except rospy.ROSInterruptException:
        pass
