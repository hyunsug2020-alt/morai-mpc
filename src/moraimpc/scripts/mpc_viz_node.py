#!/usr/bin/env python3
"""
MPC 경로 추종 RViz 시각화 노드 — 실시간 경로일치율 표시

퍼블리시 토픽:
  /mpc_viz/planned_path   - 계획 경로 (흰 점선)
  /mpc_viz/trajectory     - 실제 궤적 (|CTE| 기반 색상: 녹→적)
  /mpc_viz/recov_pts      - RECOV 구간 (주황 구)
  /mpc_viz/ego_arrow      - 현재 차량 위치/방향
  /mpc_viz/cte_text       - 차량 위 실시간 CTE/헤딩/일치율 텍스트
  /mpc_viz/stats_text     - 화면 고정 누적 통계 텍스트

서브스크라이브:
  /Ego_topic              - 실시간 차량 상태
  /mpc_performance        - [dist, solve_ms, cte, hdg_deg, v_kmh, max_kappa, target_vel]
  /mpc_status             - 현재 모드 문자열
"""
import rospy, os, json, math, collections
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
    return ColorRGBA(r=r, g=max(g, 0.0), b=0.0, a=0.9)


def match_color(pct):
    """경로일치율 % → 색상"""
    if pct >= 90:
        return ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0)  # 녹
    elif pct >= 70:
        return ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # 황
    else:
        return ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)  # 적


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
        self.cur_hdg = 0.0
        self.cur_v   = 0.0
        self.cur_target_v = 0.0
        self.cur_mode = "—"
        self.cur_solve = 0.0
        self.ego_rcvd = False

        # 실시간 누적 통계
        self.cte_history = collections.deque(maxlen=50000)
        self.hdg_history = collections.deque(maxlen=50000)
        self.total_ticks  = 0
        self.ticks_10cm   = 0   # |CTE| < 0.10m
        self.ticks_20cm   = 0   # |CTE| < 0.20m
        self.ticks_30cm   = 0   # |CTE| < 0.30m
        self.ticks_50cm   = 0   # |CTE| < 0.50m
        self.cte_sq_sum   = 0.0
        self.hdg_sq_sum   = 0.0
        self.recov_ticks  = 0
        self.start_time   = None

        rospy.Subscriber("/Ego_topic",       EgoVehicleStatus,   self._ego_cb)
        rospy.Subscriber("/mpc_performance", Float32MultiArray,  self._perf_cb)
        rospy.Subscriber("/mpc_status",      String,             self._status_cb)

        # 계획 경로 최초 퍼블리시
        self._pub_planned_path(wp_path)

        # 로그 주기적 갱신
        self._last_log_mtime = 0
        rospy.Timer(rospy.Duration(self.reload_sec), self._reload_log)

        # 실시간 마커 퍼블리시 (10Hz)
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

        m = make_marker("planned", 0, Marker.LINE_STRIP)
        m.scale.x = 0.08
        m.color   = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.7)
        for w in wps:
            p = Point(); p.x = w["x"]; p.y = w["y"]; p.z = 0.05
            m.points.append(p)
        ma.markers.append(m)

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

        # 실시간 경로일치율 계산
        pct_10 = (self.ticks_10cm / self.total_ticks * 100) if self.total_ticks > 0 else 0
        pct_20 = (self.ticks_20cm / self.total_ticks * 100) if self.total_ticks > 0 else 0
        pct_30 = (self.ticks_30cm / self.total_ticks * 100) if self.total_ticks > 0 else 0
        pct_50 = (self.ticks_50cm / self.total_ticks * 100) if self.total_ticks > 0 else 0
        cte_rmse = math.sqrt(self.cte_sq_sum / self.total_ticks) if self.total_ticks > 0 else 0
        hdg_rmse = math.sqrt(self.hdg_sq_sum / self.total_ticks) if self.total_ticks > 0 else 0

        elapsed = 0.0
        if self.start_time is not None:
            elapsed = (now - self.start_time).to_sec()

        # 차량 위 실시간 텍스트 (큰 글자)
        m2 = make_marker("cte_txt", 0, Marker.TEXT_VIEW_FACING)
        m2.header.stamp    = now
        m2.pose.position.x = self.cur_x
        m2.pose.position.y = self.cur_y
        m2.pose.position.z = 2.0
        m2.scale.z = 0.7
        m2.color = match_color(pct_20)
        m2.text  = (
            f"Match: {pct_20:.0f}%\n"
            f"CTE: {self.cur_cte:+.3f}m  HDG: {self.cur_hdg:+.1f}\xb0\n"
            f"{self.cur_v:.0f}km/h [{self.cur_mode}]"
        )
        self.pub_cte.publish(m2)

        # 화면 고정 누적 통계 텍스트
        m3 = make_marker("stats", 0, Marker.TEXT_VIEW_FACING)
        m3.header.stamp = now
        m3.pose.position.x = self.cur_x - 8.0
        m3.pose.position.y = self.cur_y + 6.0
        m3.pose.position.z = 3.0
        m3.scale.z = 0.5
        m3.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        m3.text = (
            f"=== 실시간 경로일치율 ===\n"
            f"10cm 이내: {pct_10:.1f}%\n"
            f"20cm 이내: {pct_20:.1f}%\n"
            f"30cm 이내: {pct_30:.1f}%\n"
            f"50cm 이내: {pct_50:.1f}%\n"
            f"─────────────────\n"
            f"CTE RMSE : {cte_rmse:.3f}m\n"
            f"HDG RMSE : {hdg_rmse:.1f}\xb0\n"
            f"RECOV    : {self.recov_ticks}/{self.total_ticks}\n"
            f"주행시간 : {elapsed:.0f}s\n"
            f"solve    : {self.cur_solve:.1f}ms"
        )
        self.pub_stats.publish(m3)

    # ── 콜백 ──────────────────────────────────────────────────────────
    def _ego_cb(self, msg):
        self.cur_x   = msg.position.x
        self.cur_y   = msg.position.y
        self.cur_yaw = msg.heading * math.pi / 180.0
        self.ego_rcvd = True

    def _perf_cb(self, msg):
        if len(msg.data) < 7:
            return

        cte_signed = float(msg.data[2])
        hdg_deg    = float(msg.data[3])
        v_kmh      = float(msg.data[4])
        target_v   = float(msg.data[6])
        solve_ms   = float(msg.data[1])

        self.cur_cte      = cte_signed
        self.cur_hdg      = hdg_deg
        self.cur_v        = v_kmh
        self.cur_target_v = target_v
        self.cur_solve    = solve_ms

        # 누적 통계
        cte_abs = abs(cte_signed)
        self.total_ticks += 1
        self.cte_sq_sum  += cte_signed ** 2
        self.hdg_sq_sum  += hdg_deg ** 2
        if cte_abs < 0.10: self.ticks_10cm += 1
        if cte_abs < 0.20: self.ticks_20cm += 1
        if cte_abs < 0.30: self.ticks_30cm += 1
        if cte_abs < 0.50: self.ticks_50cm += 1

        if self.start_time is None:
            self.start_time = rospy.Time.now()

    def _status_cb(self, msg):
        self.cur_mode = msg.data
        if msg.data == "RECOV":
            self.recov_ticks += 1


if __name__ == "__main__":
    MpcVizNode()
