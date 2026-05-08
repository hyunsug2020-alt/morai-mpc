#!/usr/bin/env python3
"""수동 주행 웨이포인트 기록 스크립트.

MORAI에서 키보드/조이스틱으로 직접 주행하면서
일정 간격마다 위치(x,y), heading, gear(D/R)를 기록.

사용법:
  1. MORAI를 키보드 모드(ctrl_mode=1)로 전환
  2. rosrun moraimpc record_waypoints.py
  3. 키보드로 주행 (전진/후진 포함)
  4. Ctrl+C로 종료 → JSON 파일 저장

파라미터:
  _interval:=0.5     기록 간격 [m] (기본 0.5m)
  _output:=경로       출력 파일 (기본: data/waypoints_recorded.json)
"""
import os
import json
import math
import rospy
from morai_msgs.msg import EgoVehicleStatus, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv

class WaypointRecorder:
    def __init__(self):
        rospy.init_node('waypoint_recorder')

        self.interval = rospy.get_param('~interval', 0.5)  # m
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        default_out = os.path.join(pkg_dir, 'data', 'waypoints_recorded.json')
        self.output = rospy.get_param('~output', default_out)

        self.waypoints = []
        self.prev_x = None
        self.prev_y = None
        self.cur_gear = "D"  # 현재 기어 상태

        # 키보드 모드로 전환 (ctrl_mode=1)
        rospy.loginfo("키보드 모드(ctrl_mode=1) 전환 시도...")
        try:
            rospy.wait_for_service('/Service_MoraiEventCmd', timeout=3.0)
            srv = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)
            ev = EventInfo()
            ev.option = 1  # ctrl_mode 적용
            ev.ctrl_mode = 1  # 키보드 모드
            resp = srv(ev)
            rospy.loginfo("키보드 모드 전환 완료")
        except:
            rospy.logwarn("서비스 연결 실패 — MORAI에서 수동으로 키보드 모드 설정 필요")

        self.ego_sub = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_cb)

        rospy.loginfo("=== 웨이포인트 기록 시작 ===")
        rospy.loginfo("간격: %.2f m | 출력: %s", self.interval, self.output)
        rospy.loginfo("MORAI에서 키보드로 주행하세요. Ctrl+C로 종료+저장.")

    def ego_cb(self, msg):
        x = msg.position.x
        y = msg.position.y
        heading = msg.heading  # degree

        # 속도로 기어 추정: velocity.x < -0.3 이면 후진
        vx = msg.velocity.x
        if vx < -0.3:
            self.cur_gear = "R"
        elif vx > 0.3:
            self.cur_gear = "D"
        # 정지 중이면 이전 기어 유지

        # 간격 체크
        if self.prev_x is not None:
            dist = math.hypot(x - self.prev_x, y - self.prev_y)
            if dist < self.interval:
                return

        self.prev_x = x
        self.prev_y = y

        wp = {
            "x": round(x, 6),
            "y": round(y, 6),
            "heading": round(heading * math.pi / 180.0, 8),
            "gear": self.cur_gear
        }
        self.waypoints.append(wp)

        n = len(self.waypoints)
        if n % 10 == 0:
            rospy.loginfo("기록: %d점 | (%.1f, %.1f) gear=%s", n, x, y, self.cur_gear)

    def save(self):
        if not self.waypoints:
            rospy.logwarn("기록된 웨이포인트 없음")
            return

        # D만 있으면 gear 필드 제거 (호환성)
        gears = set(wp["gear"] for wp in self.waypoints)
        if gears == {"D"}:
            for wp in self.waypoints:
                del wp["gear"]
            rospy.loginfo("전진만 기록됨 — gear 필드 생략")

        data = {"waypoints": self.waypoints}
        os.makedirs(os.path.dirname(self.output), exist_ok=True)
        with open(self.output, 'w') as f:
            json.dump(data, f, indent=2)
        rospy.loginfo("=== 저장 완료: %s (%d점) ===", self.output, len(self.waypoints))

    def run(self):
        rospy.on_shutdown(self.save)
        rospy.spin()

if __name__ == '__main__':
    WaypointRecorder().run()
