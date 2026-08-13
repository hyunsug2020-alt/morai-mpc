#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""curve_test — planner 대신 '점점 급해지는 아크' 경로를 발행해 follower의 급커브 추종정밀도 측정.
   ego 첫 위치/헤딩서 시작: 직선20m → R=20 → R=12 → R=8 → R=6 → R=5 (각 90°, 방향교대).
   /avoid_waypoints(String JSON) + /avoid_target_vel(코너캡 a_y) 발행. vehicle_sim+follower가 폐루프 추종.
   drive_monitor가 CTE/HDG를 κ별로 집계 → 급커브 정밀도 확인.
"""
import json, math
import rospy
from std_msgs.msg import String, Float32
from morai_msgs.msg import EgoVehicleStatus


class CurveTest:
    def __init__(self):
        rospy.init_node("curve_test")
        self.ay = float(rospy.get_param("~a_lat", 2.5))          # 코너캡 횡가속 [m/s²]
        self.vmax = float(rospy.get_param("~cruise_mps", 40/3.6))
        self.radii = [float(x) for x in str(rospy.get_param("~radii", "20,12,8,6,5")).split(",")]
        self.ego0 = None
        rospy.Subscriber("/localization/ego_status",
                         EgoVehicleStatus, self._ego, queue_size=1)
        self.pub_wp = rospy.Publisher("/avoid_waypoints", String, queue_size=1)
        self.pub_v = rospy.Publisher("/avoid_target_vel", Float32, queue_size=1)
        self.pub_path_str = None
        self.path = None
        rospy.loginfo("[curve_test] 대기: ego 첫 위치...")

    def _ego(self, m):
        self.ego = (m.position.x, m.position.y, math.radians(m.heading))
        self._v_est = math.hypot(m.velocity.x, m.velocity.y)
        if self.ego0 is None:
            self.ego0 = self.ego
            self._build()
            rospy.loginfo("[curve_test] 경로생성 완료 %d점 (R=%s)", len(self.path), self.radii)

    def _build(self):
        ex, ey, eh = self.ego0
        pts = []
        x, y, th = ex, ey, eh
        step = 0.5
        # 직선 15m
        for _ in range(int(15/step)):
            x += step*math.cos(th); y += step*math.sin(th); pts.append((x, y, th))
        # 아크들 (방향 교대: 좌우좌우)
        sign = 1.0
        for R in self.radii:
            arc = R*math.pi/2.0                     # 90°
            dth = step/R * sign
            for _ in range(int(arc/step)):
                th += dth
                x += step*math.cos(th); y += step*math.sin(th); pts.append((x, y, th))
            # 아크 사이 직선 25m (정상상태 안정 — S커플링 제거)
            for _ in range(int(25/step)):
                x += step*math.cos(th); y += step*math.sin(th); pts.append((x, y, th))
            sign = -sign
        self.path = pts

    def _target_v(self):
        # ego 위치 → 경로 최근접 → 전방 20m 최소R 코너캡 (구간별 realistic 속도)
        if not self.path or not hasattr(self, 'ego'):
            return self.vmax
        ex, ey, _ = self.ego
        bi = min(range(len(self.path)), key=lambda i: (self.path[i][0]-ex)**2 + (self.path[i][1]-ey)**2)
        # planner와 동일 전방탐색: V²/(2·b_comf)+V·0.5 (제동거리+반응), 최소 25m
        V = max(getattr(self, '_v_est', 11.0), 3.0)
        look = max(25.0, V*V/(2.0*3.0) + V*0.5)
        st = 6
        maxk = 0.0
        s = 0.0
        for i in range(bi+st, len(self.path)-st):
            s += math.hypot(self.path[i][0]-self.path[i-1][0], self.path[i][1]-self.path[i-1][1])
            if s > look: break
            a = self.path[i-st]; b = self.path[i]; c = self.path[i+st]
            la = math.hypot(b[0]-a[0], b[1]-a[1]); lb = math.hypot(c[0]-b[0], c[1]-b[1]); lc = math.hypot(c[0]-a[0], c[1]-a[1])
            if la*lb*lc < 1e-6: continue
            k = 2*abs((b[0]-a[0])*(c[1]-a[1])-(c[0]-a[0])*(b[1]-a[1]))/(la*lb*lc)
            maxk = max(maxk, k)
        v = self.vmax if maxk < 1e-3 else min(self.vmax, math.sqrt(self.ay/maxk))
        return v

    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.path:
                wps = [{"x": p[0], "y": p[1], "heading": p[2], "gear": "D"} for p in self.path]
                self.pub_wp.publish(String(data=json.dumps({"waypoints": wps})))
                # 속도: 전방 최소R 코너캡 (구간별로 하고 싶으면 ego투영 필요 — 여기선 보수적 전체캡)
                self.pub_v.publish(Float32(data=self._target_v()))
            r.sleep()


if __name__ == "__main__":
    try:
        CurveTest().spin()
    except rospy.ROSInterruptException:
        pass
