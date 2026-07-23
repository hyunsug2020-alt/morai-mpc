#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drive_monitor — 폐루프 주행 실시간 추종품질 진단 (비침습, 관찰전용).

  구독: /localization/ego_status, /avoid_waypoints(planner경로), /ctrl_cmd(조향/속도), /avoid_target_vel, /mpc_status
  계산: CTE(ego→경로 최근접), heading오차(ego vs 경로접선), 조향지령/조향반전율(진동),
        전방곡률, 실제vs지령속도, solve모드
  출력: 1초마다 롤링 통계 + 불안정 플래그. /drive_diag(String)로도 발행(rqt/foxglove).
"""
import json, math, collections
import rospy
from std_msgs.msg import String, Float32
from morai_msgs.msg import EgoVehicleStatus, CtrlCmd


def ang_norm(a): return (a + math.pi) % (2 * math.pi) - math.pi


class Mon:
    def __init__(self):
        rospy.init_node("drive_monitor")
        self.ego = None          # (x,y,yaw,v)
        self.path = []           # [(x,y),...]
        self.steer = 0.0         # rad
        self.velcmd = 0.0        # km/h (ctrl_cmd)
        self.tgtvel = 0.0        # m/s (avoid_target_vel)
        self.status = "?"
        W = int(rospy.get_param("~win", 40))       # 롤링창 (2s @20Hz)
        self.cte = collections.deque(maxlen=W)
        self.hderr = collections.deque(maxlen=W)
        self.steers = collections.deque(maxlen=W)
        self.vs = collections.deque(maxlen=W)
        self.steer_signs = collections.deque(maxlen=W)
        self.reversals = 0
        rospy.Subscriber("/localization/ego_status",
                         EgoVehicleStatus, self._ego, queue_size=1)
        rospy.Subscriber("/avoid_waypoints", String, self._wp, queue_size=1)
        rospy.Subscriber("/ctrl_cmd", CtrlCmd, self._cc, queue_size=1)
        rospy.Subscriber("/avoid_target_vel", Float32, lambda m: setattr(self, 'tgtvel', m.data), queue_size=1)
        rospy.Subscriber("/mpc_status", String, lambda m: setattr(self, 'status', m.data), queue_size=1)
        self.diag = rospy.Publisher("/drive_diag", String, queue_size=1)
        self.t0 = rospy.get_time()
        self.csv = open(rospy.get_param("~csv", "/tmp/drive_trace.csv"), "w")
        self.csv.write("t,status,ex,ey,eyaw,v_kmh,cmd_kmh,avoid_target_kmh,cte,hderr_deg,steer_deg,kappa\n")
        rospy.loginfo("[drive_monitor] ON (csv=%s)", self.csv.name)

    def _ego(self, m):
        self.ego = (m.position.x, m.position.y, math.radians(m.heading),
                    math.hypot(m.velocity.x, m.velocity.y))
        self._tick()

    def _wp(self, m):
        try:
            wp = json.loads(m.data)["waypoints"]
            self.path = [(w["x"], w["y"]) for w in wp]
        except Exception:
            pass

    def _cc(self, m):
        s = float(m.steering)
        self.steer = s
        self.velcmd = float(m.velocity)
        self.steers.append(s)
        sg = 1 if s > 0.01 else (-1 if s < -0.01 else 0)
        if self.steer_signs and sg != 0 and self.steer_signs[-1] != 0 and sg != self.steer_signs[-1]:
            self.reversals += 1
        self.steer_signs.append(sg)

    def _nearest(self):
        ex, ey, eyaw, ev = self.ego
        best, bi = 1e18, 0
        for i, (px, py) in enumerate(self.path):
            d = (px - ex) ** 2 + (py - ey) ** 2
            if d < best:
                best, bi = d, i
        cte = math.sqrt(best)
        # 경로 접선 heading (nearest → +2 point)
        j = min(bi + 2, len(self.path) - 1)
        if j > bi:
            pth = math.atan2(self.path[j][1] - self.path[bi][1],
                             self.path[j][0] - self.path[bi][0])
            hderr = ang_norm(eyaw - pth)
        else:
            hderr = 0.0
        # 전방 15m 최대곡률
        mk = 0.0
        s = 0.0
        for i in range(bi + 1, len(self.path) - 1):
            s += math.hypot(self.path[i][0] - self.path[i - 1][0], self.path[i][1] - self.path[i - 1][1])
            if s > 15.0:
                break
            a = self.path[max(0, i - 3)]; b = self.path[i]; c = self.path[min(len(self.path) - 1, i + 3)]
            la = math.hypot(b[0] - a[0], b[1] - a[1]); lb = math.hypot(c[0] - b[0], c[1] - b[1]); lc = math.hypot(c[0] - a[0], c[1] - a[1])
            if la * lb * lc < 1e-6:
                continue
            k = 2 * abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / (la * lb * lc)
            mk = max(mk, k)
        return cte, hderr, mk

    def _tick(self):
        if self.ego is None or len(self.path) < 3:
            return
        cte, hderr, mk = self._nearest()
        self.cte.append(cte)
        self.hderr.append(abs(math.degrees(hderr)))
        self.vs.append(self.ego[3])
        now = rospy.get_time()
        ex, ey, eyaw, ev = self.ego
        self.csv.write("%.2f,%s,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.3f,%.1f,%.2f,%.4f\n" % (
            now, self.status, ex, ey, math.degrees(eyaw), ev * 3.6,
            self.velcmd, self.tgtvel * 3.6, cte, math.degrees(hderr),
            math.degrees(self.steer), mk))
        if now - self.t0 >= 1.0:
            self.t0 = now
            n = len(self.cte)
            if n == 0:
                return
            cte_mean = sum(self.cte) / n
            cte_max = max(self.cte)
            hd_mean = sum(self.hderr) / len(self.hderr)
            hd_max = max(self.hderr)
            v = self.ego[3] * 3.6
            steer_deg = math.degrees(self.steer)
            # 조향진동: 창내 부호반전 횟수 (2s창서 >6이면 진동)
            rev = self.reversals
            self.reversals = 0
            flag = ""
            if cte_max > 1.0: flag += " ⚠CTE"
            if hd_max > 20.0: flag += " ⚠HDG"
            if rev > 6: flag += " ⚠조향진동"
            if v < 2.0: flag += " ⚠정지"
            msg = ("[diag %5.0fs] mode=%-8s v=%4.1fkm/h(cmd%4.1f) | CTE avg=%.2f max=%.2f | "
                   "HDG avg=%.1f max=%.1f° | steer=%+5.1f° rev=%d | κ전방=%.3f(코너%2.0f)%s"
                   % (now, self.status, v, self.velcmd, cte_mean, cte_max, hd_mean, hd_max,
                      steer_deg, rev, mk, (math.sqrt(2.5 / mk) * 3.6) if mk > 1e-3 else 99, flag))
            rospy.loginfo(msg)
            self.diag.publish(String(data=msg))


if __name__ == "__main__":
    try:
        Mon()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
