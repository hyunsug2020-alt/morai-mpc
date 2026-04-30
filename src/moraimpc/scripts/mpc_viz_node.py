#!/usr/bin/env python3
"""
MPC 경로 추종 RViz 시각화 노드

퍼블리시 토픽:
  /mpc_viz/planned_path   - 계획 경로 (흰 점선)
  /mpc_viz/trajectory     - 실제 궤적 (|CTE| 기반 색상: 녹→적)
  /mpc_viz/recov_pts      - RECOV 구간 (주황 구)
  /mpc_viz/ego_arrow      - 현재 차량 위치/방향
  /mpc_viz/cte_text       - 차량 위 실시간 CTE/헤딩 텍스트
  /mpc_viz/stats_text     - 화면 고정 통계 텍스트

서브스크라이브:
  /Ego_topic              - 실시간 차량 상태
  /mpc_performance        - [dist, solve_ms, |cte|]
  /mpc_status             - 현재 모드 문자열
"""
import rospy, os, json, math
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float32MultiArray, String
from morai_msgs.msg import EgoVehicleStatus

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
PKG_DIR     = os.path.dirname(SCRIPT_DIR)
DEFAULT_LOG = os.path.join(PKG_DIR, "logs", "mpc_log.json")
DEFAULT_WP  = os.path.join(PKG_DIR, "data", "waypoints.json")


def cte_to_color(cte_abs, max_cte=1.5):
    """|CTE| → RGBA 색상 (녹→황→적)"""
    r = min(1.0, cte_abs / max_cte * 2.0)
    g = min(1.0, 2.0 - cte_abs / max_cte * 2.0)
    return ColorRGBA(r=r, g=g, b=0.0, a=0.9)


def make_marker(ns, mid, mtype, frame="map"):
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp    = rospy.Time.now()
    m.ns   = ns
    m.id   = mid
    m.type = mtype
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    return m


class MpcVizNode:
    def __init__(self):
        rospy.init_node("mpc_viz_node")

        log_path = rospy.get_param("~log_file",  DEFAULT_LOG)
        wp_path  = rospy.get_param("~path_file", DEFAULT_WP)
        self.log_path    = log_path
        self.reload_sec  = rospy.get_param("~reload_sec", 3.0)

        # Publishers (latched)
        latch = True
        self.pub_plan  = rospy.Publisher("/mpc_viz/planned_path",  MarkerArray, queue_size=1, latch=latch)
        self.pub_traj  = rospy.Publisher("/mpc_viz/trajectory",    MarkerArray, queue_size=1, latch=latch)
        self.pub_recov = rospy.Publisher("/mpc_viz/recov_pts",     MarkerArray, queue_size=1, latch=latch)
        self.pub_stats = rospy.Publisher("/mpc_viz/stats_text",    Marker,      queue_size=1, latch=latch)
        self.pub_ego   = rospy.Publisher("/mpc_viz/ego_arrow",     Marker,      queue_size=1)
        self.pub_cte   = rospy.Publisher("/mpc_viz/cte_text",      Marker,      queue_size=1)

        # 실시간 상태
        self.cur_x = self.cur_y = self.cur_yaw = 0.0
        self.cur_cte = 0.0
        self.cur_mode = "—"
        self.cur_solve = 0.0
        self.ego_rcvd = False

        rospy.Subscriber("/Ego_topic",       EgoVehicleStatus,   self._ego_cb)
        rospy.Subscriber("/mpc_performance", Float32MultiArray,  self._perf_cb)
        rospy.Subscriber("/mpc_status",      String,             self._status_cb)

        # 계획 경로 최초 퍼블리시
        self._pub_planned_path(wp_path)

        # 로그 주기적 갱신
        self._last_log_mtime = 0
        rospy.Timer(rospy.Duration(self.reload_sec), self._reload_log)

        # 실시간 마커 퍼블리시
        rospy.Timer(rospy.Duration(0.1), self._pub_realtime)

        rospy.loginfo("[mpc_viz] 시작  log=%s", log_path)
        rospy.spin()

    # ── 계획 경로 ─────────────────────────────────────────────────────
    def _pub_planned_path(self, wp_path):
        if not os.path.exists(wp_path):
            rospy.logwarn("[mpc_viz] 웨이포인트 없음: %s", wp_path)
            return
        with open(wp_path) as f:
            wps = json.load(f)["waypoints"]

        ma = MarkerArray()

        # 흰 점선 (LINE_STRIP)
        m = make_marker("planned", 0, Marker.LINE_STRIP)
        m.scale.x = 0.08
        m.color   = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.7)
        for w in wps:
            p = Point(); p.x = w["x"]; p.y = w["y"]; p.z = 0.05
            m.points.append(p)
        ma.markers.append(m)

        # 웨이포인트 점
        m2 = make_marker("planned", 1, Marker.SPHERE_LIST)
        m2.scale.x = m2.scale.y = m2.scale.z = 0.2
        m2.color   = ColorRGBA(r=0.5, g=0.5, b=1.0, a=0.6)
        for w in wps:
            p = Point(); p.x = w["x"]; p.y = w["y"]; p.z = 0.05
            m2.points.append(p)
        ma.markers.append(m2)

        self.pub_plan.publish(ma)
        rospy.loginfo("[mpc_viz] 계획 경로 %d개 포인트 퍼블리시", len(wps))

    # ── 로그 리로드 & 궤적 퍼블리시 ───────────────────────────────────
    def _reload_log(self, _event=None):
        if not os.path.exists(self.log_path):
            return
        mtime = os.path.getmtime(self.log_path)
        if mtime == self._last_log_mtime:
            return
        self._last_log_mtime = mtime

        try:
            with open(self.log_path) as f:
                records = json.load(f)["records"]
        except Exception as e:
            rospy.logwarn("[mpc_viz] 로그 읽기 실패: %s", e)
            return

        self._pub_trajectory(records)
        self._pub_recov(records)
        self._pub_stats_text(records)

    def _pub_trajectory(self, records):
        ma   = MarkerArray()
        pts  = []
        cols = []
        for r in records:
            if r.get("mode") in ("RECOV", "HDG_RECOV", "STOP"):
                continue
            cte_abs = abs(r.get("cte", 0.0))
            pts.append((r["x"], r["y"]))
            cols.append(cte_to_color(cte_abs))

        if not pts:
            return

        # SPHERE_LIST — 점마다 색상 지정 (LINE_STRIP은 per-point 색 미지원)
        m = make_marker("traj", 0, Marker.SPHERE_LIST)
        m.scale.x = m.scale.y = m.scale.z = 0.18
        for (x, y), c in zip(pts, cols):
            p = Point(); p.x = x; p.y = y; p.z = 0.03
            m.points.append(p)
            m.colors.append(c)
        ma.markers.append(m)
        self.pub_traj.publish(ma)

    def _pub_recov(self, records):
        ma = MarkerArray()
        m  = make_marker("recov", 0, Marker.SPHERE_LIST)
        m.scale.x = m.scale.y = m.scale.z = 0.35
        m.color   = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
        for r in records:
            if r.get("mode") not in ("RECOV", "HDG_RECOV"):
                continue
            p = Point(); p.x = r["x"]; p.y = r["y"]; p.z = 0.1
            m.points.append(p)
        if m.points:
            ma.markers.append(m)
        self.pub_recov.publish(ma)

    def _pub_stats_text(self, records):
        if not records:
            return
        cte_arr  = np.array([r.get("cte", 0) for r in records])
        cte_abs  = np.abs(cte_arr)
        n        = len(records)
        rmse     = math.sqrt(float(np.mean(cte_arr ** 2)))
        p30      = float((cte_abs < 0.30).sum()) / n * 100
        p10      = float((cte_abs < 0.10).sum()) / n * 100
        n_recov  = sum(1 for r in records if r.get("mode") == "RECOV")
        duration = float(records[-1].get("t", 0))
        sm_arr   = [r.get("solve_ms", 0) for r in records if r.get("solve_ms")]
        sm_mean  = sum(sm_arr) / len(sm_arr) if sm_arr else 0

        text = (
            f"[경로 일치율]\n"
            f"CTE RMSE : {rmse:.3f} m\n"
            f"10cm 이내: {p10:.1f}%\n"
            f"30cm 이내: {p30:.1f}%\n"
            f"RECOV    : {n_recov}/{n} ({100*n_recov/n:.1f}%)\n"
            f"solve ms : {sm_mean:.1f} ms\n"
            f"주행시간 : {duration:.1f} s"
        )

        # 첫 번째 웨이포인트 근처 고정 위치에 표시
        ref_x = records[0].get("x", 0)
        ref_y = records[0].get("y", 0)

        m = make_marker("stats", 0, Marker.TEXT_VIEW_FACING)
        m.pose.position.x = ref_x - 5.0
        m.pose.position.y = ref_y + 5.0
        m.pose.position.z = 2.0
        m.scale.z   = 0.6
        m.color     = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        m.text      = text
        self.pub_stats.publish(m)

    # ── 실시간 마커 ───────────────────────────────────────────────────
    def _pub_realtime(self, _event=None):
        if not self.ego_rcvd:
            return
        now = rospy.Time.now()

        # 차량 화살표
        m = make_marker("ego", 0, Marker.ARROW)
        m.header.stamp  = now
        m.pose.position.x = self.cur_x
        m.pose.position.y = self.cur_y
        m.pose.position.z = 0.3
        import tf.transformations as tft
        q = tft.quaternion_from_euler(0, 0, self.cur_yaw)
        m.pose.orientation.x = q[0]; m.pose.orientation.y = q[1]
        m.pose.orientation.z = q[2]; m.pose.orientation.w = q[3]
        m.scale.x = 1.8; m.scale.y = 0.4; m.scale.z = 0.4
        mode_c = {
            "NORMAL":    ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0),
            "RECOV":     ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0),
            "HDG_RECOV": ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0),
        }
        m.color = mode_c.get(self.cur_mode, ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0))
        self.pub_ego.publish(m)

        # CTE 텍스트
        m2 = make_marker("cte_txt", 0, Marker.TEXT_VIEW_FACING)
        m2.header.stamp    = now
        m2.pose.position.x = self.cur_x
        m2.pose.position.y = self.cur_y
        m2.pose.position.z = 1.5
        m2.scale.z = 0.5
        cte_c = cte_to_color(abs(self.cur_cte))
        m2.color = cte_c
        m2.text  = (f"CTE:{self.cur_cte:+.2f}m  [{self.cur_mode}]\n"
                    f"solve:{self.cur_solve:.0f}ms")
        self.pub_cte.publish(m2)

    # ── 콜백 ──────────────────────────────────────────────────────────
    def _ego_cb(self, msg):
        self.cur_x   = msg.position.x
        self.cur_y   = msg.position.y
        self.cur_yaw = msg.heading * math.pi / 180.0
        self.ego_rcvd = True

    def _perf_cb(self, msg):
        if len(msg.data) >= 3:
            self.cur_cte   = msg.data[2] * (1 if self.cur_cte >= 0 else -1)
            self.cur_solve = msg.data[1]

    def _status_cb(self, msg):
        self.cur_mode = msg.data


if __name__ == "__main__":
    MpcVizNode()
