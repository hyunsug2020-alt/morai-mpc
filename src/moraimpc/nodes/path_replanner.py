#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
path_replanner.py — 시작 시 1회 실행: /Object_topic NPC 위치 받아 mixed.json path에
NPC 옆을 우회하는 lateral shift bulge 적용 → 새 path 파일 저장 후 종료.

path_follower가 이 새 파일을 사용하면 자동 회피 + 복귀 (NMPC 추종은 변경 path 따라감).
"""
import json
import math
import os
import rospy
from morai_msgs.msg import ObjectStatusList


class PathReplanner:
    def __init__(self):
        rospy.init_node('path_replanner')
        self.in_path = rospy.get_param('~in_path',
            os.path.expanduser('~/morai-mpc/src/moraimpc/data/mixed.json'))
        self.out_path = rospy.get_param('~out_path',
            os.path.expanduser('~/morai-mpc/src/moraimpc/data/mixed_avoid.json'))
        self.shift_max = rospy.get_param('~shift_max', 2.5)        # ±2.5m lateral 우회
        self.bulge_half_m = rospy.get_param('~bulge_half_m', 12.0)  # NPC 중심 ±12m smooth
        self.lat_thresh = rospy.get_param('~lat_thresh', 1.5)      # path 위 NPC 판정 lateral d
        self.wait_npcs_sec = rospy.get_param('~wait_npcs_sec', 3.0)

        self.objs = None
        rospy.Subscriber('/Object_topic', ObjectStatusList, self._obj_cb, queue_size=1)

        rospy.loginfo(f"[replanner] in={self.in_path}  out={self.out_path}")
        rospy.loginfo(f"[replanner] /Object_topic 대기 {self.wait_npcs_sec}초")
        end_t = rospy.Time.now() + rospy.Duration(self.wait_npcs_sec)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < end_t:
            if self.objs is not None and self.objs.npc_list:
                break
            rate.sleep()

        wps = self._load_waypoints()
        if self.objs is None or not self.objs.npc_list:
            rospy.logwarn("[replanner] NPC 없음 → 원본 그대로 저장")
            self._save_waypoints(wps)
            return

        rospy.loginfo(f"[replanner] NPC {len(self.objs.npc_list)}대 감지 — path 우회 생성")
        new_wps = self._apply_bulge(wps, self.objs.npc_list)
        self._save_waypoints(new_wps)
        rospy.loginfo(f"[replanner] 저장 완료 → {self.out_path}")

    def _obj_cb(self, msg):
        self.objs = msg

    def _load_waypoints(self):
        with open(self.in_path) as f:
            d = json.load(f)
        return d['waypoints'] if isinstance(d, dict) else d

    def _save_waypoints(self, wps):
        with open(self.out_path, 'w') as f:
            json.dump({'waypoints': wps}, f, indent=2)

    @staticmethod
    def _path_yaw(wps, i):
        i1 = min(i + 1, len(wps) - 1)
        return math.atan2(wps[i1]['y'] - wps[i]['y'], wps[i1]['x'] - wps[i]['x'])

    def _nearest_idx(self, wps, x, y):
        best_d2, best = 1e18, 0
        for i, w in enumerate(wps):
            dx, dy = w['x'] - x, w['y'] - y
            dd = dx*dx + dy*dy
            if dd < best_d2:
                best_d2 = dd
                best = i
        return best

    def _apply_bulge(self, wps, npcs):
        n = len(wps)
        wp_spacing = max(0.3, math.hypot(wps[1]['x'] - wps[0]['x'], wps[1]['y'] - wps[0]['y']))
        half_wp = max(20, int(self.bulge_half_m / wp_spacing))
        bulges = []
        for npc in npcs:
            ox, oy = npc.position.x, npc.position.y
            idx = self._nearest_idx(wps, ox, oy)
            th = self._path_yaw(wps, idx)
            rx, ry = ox - wps[idx]['x'], oy - wps[idx]['y']
            d_npc = -math.sin(th) * rx + math.cos(th) * ry
            if abs(d_npc) > self.lat_thresh:
                rospy.loginfo(f"[replanner] NPC ({ox:.1f},{oy:.1f}) d={d_npc:+.2f}m path 옆 → skip")
                continue
            shift = -self.shift_max if d_npc >= 0 else self.shift_max
            rospy.loginfo(f"[replanner] NPC ({ox:.1f},{oy:.1f}) wp[{idx}] d={d_npc:+.2f}m shift={shift:+.2f}")
            bulges.append((idx, shift))

        new = []
        for i, w in enumerate(wps):
            total = 0.0
            for c_idx, sh in bulges:
                dist = abs(i - c_idx)
                if dist >= half_wp:
                    continue
                weight = 0.5 * (1.0 + math.cos(math.pi * dist / half_wp))
                total += sh * weight
            th = self._path_yaw(wps, i)
            nx = w['x'] + total * (-math.sin(th))
            ny = w['y'] + total * ( math.cos(th))
            nw = dict(w)
            nw['x'] = nx
            nw['y'] = ny
            new.append(nw)
        return new


if __name__ == '__main__':
    try:
        PathReplanner()
    except rospy.ROSInterruptException:
        pass
