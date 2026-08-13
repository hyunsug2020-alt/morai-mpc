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
        self.bulge_half_m = rospy.get_param('~bulge_half_m', 12.0)  # 호환용 폴백
        self.bulge_half_front_m = rospy.get_param('~bulge_half_front_m', self.bulge_half_m)  # NPC 앞 진입 taper
        self.bulge_half_back_m  = rospy.get_param('~bulge_half_back_m', 5.0)                  # NPC 뒤 복귀 taper (단축)
        self.lat_thresh = rospy.get_param('~lat_thresh', 1.5)      # path 위 NPC 판정 lateral d
        self.merge_threshold_m = rospy.get_param('~merge_threshold_m', 50.0)  # plateau merge 복원 (분리 bulge → 두 번 회피 + 오버슛 야기)
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
        half_front = max(20, int(self.bulge_half_front_m / wp_spacing))
        half_back  = max(10, int(self.bulge_half_back_m  / wp_spacing))
        merge_thr_wp = int(self.merge_threshold_m / wp_spacing)
        rospy.loginfo(f"[replanner] taper front={self.bulge_half_front_m:.1f}m({half_front}wp) back={self.bulge_half_back_m:.1f}m({half_back}wp) merge_thr={self.merge_threshold_m:.1f}m")

        # 1) NPC별 (c_idx, d_signed) 계산 — 회피 방향 통일 후 shift 결정
        candidates = []
        for npc in npcs:
            ox, oy = npc.position.x, npc.position.y
            idx = self._nearest_idx(wps, ox, oy)
            th = self._path_yaw(wps, idx)
            rx, ry = ox - wps[idx]['x'], oy - wps[idx]['y']
            d_npc = -math.sin(th) * rx + math.cos(th) * ry
            if abs(d_npc) > self.lat_thresh:
                rospy.loginfo(f"[replanner] NPC ({ox:.1f},{oy:.1f}) d={d_npc:+.2f}m path 옆 → skip")
                continue
            candidates.append((idx, d_npc, ox, oy))

        # 회피 방향 majority: d 절대값 가중 (path 중앙에 가까운 NPC는 약한 신호)
        if candidates:
            score = sum(d for _, d, _, _ in candidates)
            # 모두 동일 방향으로 통일: score >= 0이면 좌측 NPC 가정 → 우측 회피(-shift), 반대도 동일
            unified_shift = -self.shift_max if score >= 0 else self.shift_max
            rospy.loginfo(f"[replanner] d_score={score:+.2f} → 통일 회피방향 shift={unified_shift:+.2f}")
        else:
            unified_shift = 0.0

        bulges = []
        for idx, d_npc, ox, oy in candidates:
            rospy.loginfo(f"[replanner] NPC ({ox:.1f},{oy:.1f}) wp[{idx}] d={d_npc:+.2f}m shift={unified_shift:+.2f}")
            bulges.append((idx, unified_shift))

        # 2) 같은 방향 인접 NPC를 plateau group으로 merge (path nearest_idx 순)
        bulges.sort(key=lambda b: b[0])
        groups = []
        for c_idx, sh in bulges:
            if groups and (c_idx - groups[-1]['end'] < merge_thr_wp) and (sh * groups[-1]['shift'] > 0):
                # 같은 방향 + 가까움 → 그룹 확장 (plateau 연장)
                groups[-1]['end'] = c_idx
                groups[-1]['members'].append(c_idx)
            else:
                groups.append({'start': c_idx, 'end': c_idx, 'shift': sh, 'members': [c_idx]})
        for g in groups:
            plat_m = (g['end'] - g['start']) * wp_spacing
            rospy.loginfo(f"[replanner]   group: wp[{g['start']}..{g['end']}] plateau={plat_m:.1f}m shift={g['shift']:+.2f} NPCs={len(g['members'])}")

        # 3) wp별 lateral offset = group 별 weight (사이 plateau=1) 합
        new = []
        for i, w in enumerate(wps):
            total = 0.0
            for g in groups:
                if i < g['start']:
                    dist = g['start'] - i
                    if dist >= half_front: continue
                    t = dist / half_front
                    weight = 1.0 - (10*t**3 - 15*t**4 + 6*t**5)
                elif i > g['end']:
                    dist = i - g['end']
                    if dist >= half_back: continue
                    t = dist / half_back
                    weight = 1.0 - (10*t**3 - 15*t**4 + 6*t**5)
                else:
                    weight = 1.0   # plateau: NPC들 사이 lateral 일정
                total += g['shift'] * weight
            # 같은 방향이면 cap, 반대 방향이면 그대로 (희귀 케이스)
            if abs(total) > self.shift_max * 1.05:
                total = math.copysign(self.shift_max, total)
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
