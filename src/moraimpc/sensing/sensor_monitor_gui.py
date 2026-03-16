#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HENES 통합 센서 모니터 GUI
============================
GPS+IMU 런치 실행 시 하나의 창에 모든 정보를 표시합니다.

┌──────────────────────────────────────────────────────────────────────────┐
│  HENES Sensor Monitor                                                    │
├─────────────────┬──────────────────────┬─────────────────────────────────┤
│   IMU           │   GPS                │   HEADING (ESKF)                │
│  (기존 GUI 포함) │  Fix / 위성 / 정확도 │  컴퍼스 + 헤딩 비교             │
├─────────────────┴──────────────────────┴─────────────────────────────────┤
│  Raw Values                                                              │
└──────────────────────────────────────────────────────────────────────────┘

구독 토픽:
  /handsfree/imu                  (sensor_msgs/Imu)
  /gnss_rover/fix                 (sensor_msgs/NavSatFix)
  /ublox_gps/fix_velocity         (geometry_msgs/TwistWithCovarianceStamped)
  /dual_f9p/heading               (std_msgs/Float64)
  /dual_f9p/heading_valid         (std_msgs/Bool)
  /dual_f9p/heading_accuracy_deg  (std_msgs/Float64)
  /dual_f9p/baseline_length_m     (std_msgs/Float64)
  /eskf/heading_deg               (std_msgs/Float64)
  /eskf/covariance_trace          (std_msgs/Float64)
  /odometry/filtered              (nav_msgs/Odometry)
  /gnss_rover/ubx_nav_pvt         (ublox_ubx_msgs/UBXNavPVT, optional)
"""

from __future__ import annotations

import math
import threading
import tkinter as tk
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy,
                        QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64

try:
    from ublox_ubx_msgs.msg import UBXNavPVT
    _HAS_UBX_PVT = True
except ImportError:
    _HAS_UBX_PVT = False

# NavSatStatus 상수
GPS_FIX_NONE   = -1
GPS_FIX_FIX    = 0
GPS_FIX_SBAS   = 1
GPS_FIX_GBAS   = 2


# ============================================================
# 스냅샷 (스레드 안전 데이터 컨테이너)
# ============================================================

@dataclass
class SensorSnapshot:
    stamp_sec: float = 0.0

    # IMU
    imu_stamp:  float = 0.0
    roll_deg:   float = 0.0
    pitch_deg:  float = 0.0
    imu_yaw_deg: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0

    # GPS Fix
    gps_stamp:  float = 0.0
    gps_status: int   = GPS_FIX_NONE  # NavSatStatus
    gps_lat:    float = 0.0
    gps_lon:    float = 0.0
    gps_h_acc:  float = 999.0   # [m] 수평 정확도
    gps_v_acc:  float = 999.0   # [m] 수직 정확도

    # GPS 속도
    vel_stamp: float = 0.0
    vel_x:     float = 0.0
    vel_y:     float = 0.0
    speed_mps: float = 0.0

    # UBXNavPVT (선택)
    pvt_stamp:    float = 0.0
    num_sv:       int   = 0       # 사용 위성 수
    fix_type:     int   = 0       # 0=no, 2=2D, 3=3D
    carr_soln:    int   = 0       # 0=none, 1=float, 2=fixed
    p_dop:        float = 99.0
    pvt_h_acc_m:  float = 999.0
    pvt_v_acc_m:  float = 999.0

    # 듀얼 GPS 헤딩
    dual_stamp:    float = 0.0
    dual_heading:  float = 0.0    # [deg]
    dual_valid:    bool  = False
    dual_accuracy: float = 99.0   # [deg] 1σ
    dual_baseline: float = 0.0    # [m]

    # ESKF
    eskf_stamp:   float = 0.0
    eskf_heading: float = 0.0    # [deg]
    eskf_cov:     float = 0.0    # 공분산 트레이스

    # Odometry (ESKF 출력)
    odom_stamp: float = 0.0
    odom_x:     float = 0.0
    odom_y:     float = 0.0
    odom_vx:    float = 0.0
    odom_vy:    float = 0.0


# ============================================================
# ROS2 노드
# ============================================================

class SensorMonitorNode(Node):

    STALE_SEC = 1.5

    def __init__(self) -> None:
        super().__init__('sensor_monitor_gui')

        self.declare_parameter('imu_topic',                '/handsfree/imu')
        self.declare_parameter('gps_fix_topic',            '/gnss_rover/fix')
        self.declare_parameter('gps_vel_topic',            '/ublox_gps/fix_velocity')
        self.declare_parameter('dual_heading_topic',       '/dual_f9p/heading')
        self.declare_parameter('dual_heading_valid_topic', '/dual_f9p/heading_valid')
        self.declare_parameter('dual_heading_acc_topic',   '/dual_f9p/heading_accuracy_deg')
        self.declare_parameter('dual_baseline_topic',      '/dual_f9p/baseline_length_m')
        self.declare_parameter('eskf_heading_topic',       '/eskf/heading_deg')
        self.declare_parameter('eskf_cov_topic',           '/eskf/covariance_trace')
        self.declare_parameter('odom_topic',               '/odometry/filtered')
        self.declare_parameter('pvt_topic',                '/gnss_rover/ubx_nav_pvt')
        self.declare_parameter('stale_timeout_sec',        self.STALE_SEC)
        self.declare_parameter('refresh_ms',               80)

        p = lambda n: self.get_parameter(n).value  # noqa
        self.STALE_SEC = float(p('stale_timeout_sec'))
        self.refresh_ms = int(p('refresh_ms'))

        self._snap = SensorSnapshot()
        self._lock = threading.Lock()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=20)

        self.create_subscription(Imu,                    p('imu_topic'),                self._imu_cb,          sensor_qos)
        self.create_subscription(NavSatFix,              p('gps_fix_topic'),            self._gps_fix_cb,      sensor_qos)
        self.create_subscription(TwistWithCovarianceStamped, p('gps_vel_topic'),        self._gps_vel_cb,      sensor_qos)
        self.create_subscription(Float64,                p('dual_heading_topic'),       self._dual_head_cb,    10)
        self.create_subscription(Bool,                   p('dual_heading_valid_topic'), self._dual_valid_cb,   10)
        self.create_subscription(Float64,                p('dual_heading_acc_topic'),   self._dual_acc_cb,     10)
        self.create_subscription(Float64,                p('dual_baseline_topic'),      self._dual_base_cb,    10)
        self.create_subscription(Float64,                p('eskf_heading_topic'),       self._eskf_head_cb,    10)
        self.create_subscription(Float64,                p('eskf_cov_topic'),           self._eskf_cov_cb,     10)
        self.create_subscription(Odometry,               p('odom_topic'),               self._odom_cb,         sensor_qos)

        if _HAS_UBX_PVT:
            from ublox_ubx_msgs.msg import UBXNavPVT
            self.create_subscription(UBXNavPVT, p('pvt_topic'), self._pvt_cb, 10)

        self.get_logger().info('센서 모니터 GUI 노드 시작')

    # ── 콜백 ──────────────────────────────────────

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _quat_to_euler(self, x, y, z, w):
        sinr = 2*(w*x+y*z); cosr = 1-2*(x*x+y*y)
        roll = math.atan2(sinr, cosr)
        sinp = 2*(w*y-z*x)
        pitch = math.asin(max(-1., min(1., sinp)))
        siny = 2*(w*z+x*y); cosy = 1-2*(y*y+z*z)
        yaw = math.atan2(siny, cosy)
        return roll, pitch, yaw

    def _imu_cb(self, msg: Imu):
        roll, pitch, yaw = self._quat_to_euler(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w)
        with self._lock:
            s = self._snap
            s.imu_stamp   = self._now()
            s.roll_deg    = math.degrees(roll)
            s.pitch_deg   = math.degrees(pitch)
            s.imu_yaw_deg = math.degrees(yaw)
            s.gyro_x = msg.angular_velocity.x
            s.gyro_y = msg.angular_velocity.y
            s.gyro_z = msg.angular_velocity.z
            s.accel_x = msg.linear_acceleration.x
            s.accel_y = msg.linear_acceleration.y
            s.accel_z = msg.linear_acceleration.z

    def _gps_fix_cb(self, msg: NavSatFix):
        h_acc = 999.0
        if len(msg.position_covariance) >= 1 and msg.position_covariance[0] > 0:
            h_acc = math.sqrt(msg.position_covariance[0])
        v_acc = 999.0
        if len(msg.position_covariance) >= 9 and msg.position_covariance[8] > 0:
            v_acc = math.sqrt(msg.position_covariance[8])
        with self._lock:
            s = self._snap
            s.gps_stamp  = self._now()
            s.gps_status = msg.status.status
            s.gps_lat    = msg.latitude
            s.gps_lon    = msg.longitude
            s.gps_h_acc  = h_acc
            s.gps_v_acc  = v_acc

    def _gps_vel_cb(self, msg: TwistWithCovarianceStamped):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        with self._lock:
            s = self._snap
            s.vel_stamp = self._now()
            s.vel_x     = vx
            s.vel_y     = vy
            s.speed_mps = math.hypot(vx, vy)

    def _dual_head_cb(self, msg: Float64):
        with self._lock:
            self._snap.dual_stamp   = self._now()
            self._snap.dual_heading = float(msg.data)

    def _dual_valid_cb(self, msg: Bool):
        with self._lock:
            self._snap.dual_valid = bool(msg.data)

    def _dual_acc_cb(self, msg: Float64):
        with self._lock:
            self._snap.dual_accuracy = float(msg.data)

    def _dual_base_cb(self, msg: Float64):
        with self._lock:
            self._snap.dual_baseline = float(msg.data)

    def _eskf_head_cb(self, msg: Float64):
        with self._lock:
            self._snap.eskf_stamp   = self._now()
            self._snap.eskf_heading = float(msg.data)

    def _eskf_cov_cb(self, msg: Float64):
        with self._lock:
            self._snap.eskf_cov = float(msg.data)

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            s = self._snap
            s.odom_stamp = self._now()
            s.odom_x  = msg.pose.pose.position.x
            s.odom_y  = msg.pose.pose.position.y
            s.odom_vx = msg.twist.twist.linear.x
            s.odom_vy = msg.twist.twist.linear.y

    def _pvt_cb(self, msg):
        with self._lock:
            s = self._snap
            s.pvt_stamp   = self._now()
            s.num_sv      = int(getattr(msg, 'num_sv',   0))
            s.fix_type    = int(getattr(msg, 'fix_type', 0))
            s.carr_soln   = int(getattr(msg, 'carr_soln',0))
            s.p_dop       = float(getattr(msg, 'p_dop', 9900)) * 0.01
            h_mm = float(getattr(msg, 'h_acc', 999000))
            v_mm = float(getattr(msg, 'v_acc', 999000))
            s.pvt_h_acc_m = h_mm * 1e-3
            s.pvt_v_acc_m = v_mm * 1e-3

    def get_snapshot(self) -> SensorSnapshot:
        with self._lock:
            from copy import copy
            return copy(self._snap)


# ============================================================
# GUI 창
# ============================================================

class SensorMonitorWindow:

    # 색상 팔레트 (기존 IMU GUI와 동일 스타일)
    BG       = '#f2ecdf'
    PANEL    = '#fffaf1'
    PANEL_H  = '#ede3cf'
    TEXT     = '#1e242b'
    MUTED    = '#6e757d'
    GOOD     = '#2c9961'
    WARN     = '#cb7d00'
    BAD      = '#be3b34'
    BLUE     = '#1664c0'
    PURPLE   = '#7d4bd6'
    CYAN     = '#0891b2'

    W = 1500
    H = 860

    def __init__(self, node: SensorMonitorNode) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title('HENES Sensor Monitor  —  IMU · GPS · Heading')
        self.root.geometry(f'{self.W}x{self.H}')
        self.root.configure(bg=self.BG)
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self.canvas = tk.Canvas(self.root, width=self.W, height=self.H,
                                bg=self.BG, highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self._closed = False

    def run(self) -> None:
        self._tick()
        self.root.mainloop()

    def _close(self) -> None:
        self._closed = True
        self.root.destroy()

    def _tick(self) -> None:
        if self._closed:
            return
        s = self.node.get_snapshot()
        now = self.node.get_clock().now().nanoseconds * 1e-9
        self._draw(s, now)
        self.root.after(self.node.refresh_ms, self._tick)

    # ── 공통 헬퍼 ─────────────────────────────────

    def _fresh(self, stamp: float, now: float) -> bool:
        return stamp > 0.0 and (now - stamp) <= self.node.STALE_SEC

    def _card(self, x, y, w, h, title):
        c = self.canvas
        c.create_rectangle(x, y, x+w, y+h, fill=self.PANEL,
                           outline='#cfbea0', width=2)
        c.create_rectangle(x, y, x+w, y+10, fill=self.PANEL_H, outline='')
        c.create_text(x+18, y+32, text=title, anchor='w',
                      font=('Helvetica', 15, 'bold'), fill=self.TEXT)

    def _pill(self, x, y, text, color, size=11):
        w = 11*len(text)+22
        self.canvas.create_rectangle(x, y, x+w, y+30, fill=color, outline='')
        self.canvas.create_text(x+w/2, y+16, text=text,
                                font=('Helvetica', size, 'bold'), fill='#fff')
        return w

    def _label_value(self, x, y, label, value, color=None):
        c = self.canvas
        c.create_text(x, y, text=label, anchor='w',
                      font=('Helvetica', 12), fill=self.MUTED)
        c.create_text(x+140, y, text=value, anchor='w',
                      font=('Courier', 13, 'bold'),
                      fill=color or self.TEXT)

    # ── 메인 드로우 ───────────────────────────────

    def _draw(self, s: SensorSnapshot, now: float) -> None:
        c = self.canvas
        c.delete('all')

        # 배경
        c.create_rectangle(0, 0, self.W, self.H, fill=self.BG, outline='')

        # 헤더 바
        c.create_rectangle(0, 0, self.W, 68, fill='#e4d7bb', outline='')
        c.create_text(28, 28, text='HENES Sensor Monitor',
                      anchor='w', font=('Helvetica', 22, 'bold'), fill=self.TEXT)
        c.create_text(28, 54, text='IMU  ·  GPS  ·  ESKF Heading',
                      anchor='w', font=('Helvetica', 11), fill=self.MUTED)

        # 전체 상태 표시
        any_fresh = (self._fresh(s.imu_stamp, now) or
                     self._fresh(s.gps_stamp, now) or
                     self._fresh(s.eskf_stamp, now))
        self._pill(self.W-160, 18, 'LIVE' if any_fresh else 'STALE',
                   self.GOOD if any_fresh else self.BAD, size=13)

        # ── 3개 패널 ──
        PAD = 12
        panel_y  = 80
        panel_h  = 560
        col_w    = (self.W - PAD*4) // 3

        x_imu    = PAD
        x_gps    = PAD*2 + col_w
        x_head   = PAD*3 + col_w*2

        self._draw_imu_panel(s, now, x_imu,  panel_y, col_w, panel_h)
        self._draw_gps_panel(s, now, x_gps,  panel_y, col_w, panel_h)
        self._draw_head_panel(s, now, x_head, panel_y, col_w, panel_h)

        # ── 하단 Raw 패널 ──
        raw_y = panel_y + panel_h + 10
        self._draw_raw_bar(s, now, PAD, raw_y, self.W - PAD*2, self.H - raw_y - PAD)

    # ── IMU 패널 ──────────────────────────────────

    def _draw_imu_panel(self, s, now, x, y, w, h):
        fresh = self._fresh(s.imu_stamp, now)
        self._card(x, y, w, h, '1. IMU')
        self._pill(x+w-130, y+12, 'LIVE' if fresh else 'STALE',
                   self.GOOD if fresh else self.BAD)

        cx = x + w//2
        ys = y + 60

        # ── Roll 시각화 ──
        label = self._roll_label(s.roll_deg)
        self._draw_tilt_bar(cx, ys+20, 160, s.roll_deg, 30.0, label)
        self.canvas.create_text(cx, ys+58, text=f'roll = {s.roll_deg:+.2f}°',
                                font=('Courier', 12, 'bold'), fill=self.TEXT)

        # ── Pitch 시각화 ──
        label2 = self._pitch_label(s.pitch_deg)
        self._draw_tilt_bar(cx, ys+95, 160, s.pitch_deg, 30.0, label2)
        self.canvas.create_text(cx, ys+133, text=f'pitch = {s.pitch_deg:+.2f}°',
                                font=('Courier', 12, 'bold'), fill=self.TEXT)

        # ── IMU Yaw ──
        col = self.PURPLE
        self.canvas.create_text(cx, ys+172,
                                text=f'yaw = {s.imu_yaw_deg:+.2f}°',
                                font=('Courier', 13, 'bold'), fill=col)

        # ── 회전 상태 ──
        turn = self._turn_label(s.gyro_z)
        tc = self.GOOD if 'CCW' in turn else (self.BAD if 'CW' in turn else self.MUTED)
        self.canvas.create_text(cx, ys+205,
                                text=turn,
                                font=('Helvetica', 12, 'bold'), fill=tc)

        # ── 수치 ──
        lx = x + 16
        self._label_value(lx, ys+245, 'gyro x',  f'{s.gyro_x:+.3f} r/s')
        self._label_value(lx, ys+268, 'gyro y',  f'{s.gyro_y:+.3f} r/s')
        self._label_value(lx, ys+291, 'gyro z',  f'{s.gyro_z:+.3f} r/s')
        self._label_value(lx, ys+322, 'accel x', f'{s.accel_x:+.3f} m/s²')
        self._label_value(lx, ys+345, 'accel y', f'{s.accel_y:+.3f} m/s²')
        self._label_value(lx, ys+368, 'accel z', f'{s.accel_z:+.3f} m/s²')

    def _roll_label(self, deg):
        if deg > 4:   return 'LEFT UP'
        if deg < -4:  return 'RIGHT UP'
        return 'LEVEL'

    def _pitch_label(self, deg):
        if deg > 4:   return 'FRONT UP'
        if deg < -4:  return 'REAR UP'
        return 'LEVEL'

    def _turn_label(self, gz):
        if gz > 0.15: return 'CCW / LEFT TURN'
        if gz < -0.15: return 'CW / RIGHT TURN'
        return 'STILL'

    def _draw_tilt_bar(self, cx, cy, bar_w, value, max_val, label):
        c = self.canvas
        half = bar_w // 2
        # 배경
        c.create_rectangle(cx-half, cy-8, cx+half, cy+8,
                           fill='#ddd6c4', outline='')
        # 채움
        ratio = max(-1.0, min(1.0, value / max_val))
        fill_w = int(abs(ratio) * half)
        col = self.GOOD if abs(value) > 4 else self.MUTED
        if ratio > 0:
            c.create_rectangle(cx, cy-6, cx+fill_w, cy+6, fill=col, outline='')
        else:
            c.create_rectangle(cx-fill_w, cy-6, cx, cy+6, fill=col, outline='')
        # 중앙선
        c.create_line(cx, cy-10, cx, cy+10, fill=self.TEXT, width=2)
        c.create_text(cx, cy-22, text=label, font=('Helvetica', 10, 'bold'),
                      fill=col)

    # ── GPS 패널 ──────────────────────────────────

    def _draw_gps_panel(self, s, now, x, y, w, h):
        fresh_fix = self._fresh(s.gps_stamp, now)
        fresh_pvt = self._fresh(s.pvt_stamp, now)
        self._card(x, y, w, h, '2. GPS')
        self._pill(x+w-130, y+12, 'LIVE' if fresh_fix else 'STALE',
                   self.GOOD if fresh_fix else self.BAD)

        cx = x + w//2
        lx = x + 16
        ys = y + 60

        # ── Fix 상태 ──
        fix_str, fix_col = self._fix_info(s)
        self.canvas.create_text(cx, ys+10,
                                text=fix_str,
                                font=('Helvetica', 17, 'bold'), fill=fix_col)

        # ── RTK 상태 (PVT 있을 경우) ──
        if fresh_pvt:
            rtk_str, rtk_col = self._rtk_info(s.carr_soln)
            self.canvas.create_text(cx, ys+38,
                                    text=f'RTK: {rtk_str}',
                                    font=('Helvetica', 13, 'bold'), fill=rtk_col)
        else:
            self.canvas.create_text(cx, ys+38,
                                    text='RTK: ---',
                                    font=('Helvetica', 13), fill=self.MUTED)

        # ── 수평 정확도 게이지 ──
        h_acc = s.pvt_h_acc_m if fresh_pvt else s.gps_h_acc
        acc_col = (self.GOOD if h_acc < 0.1 else
                   self.WARN if h_acc < 0.5 else self.BAD)
        self._draw_accuracy_gauge(cx, ys+90, 130, h_acc, 2.0, acc_col)
        self.canvas.create_text(cx, ys+120,
                                text=f'수평 정확도  {h_acc:.3f} m',
                                font=('Courier', 12, 'bold'), fill=acc_col)

        # ── 수직 정확도 ──
        v_acc = s.pvt_v_acc_m if fresh_pvt else s.gps_v_acc
        vacc_col = (self.GOOD if v_acc < 0.2 else
                    self.WARN if v_acc < 1.0 else self.BAD)
        self.canvas.create_text(cx, ys+148,
                                text=f'수직 정확도  {v_acc:.3f} m',
                                font=('Courier', 12, 'bold'), fill=vacc_col)

        # ── 위성 수 ──
        if fresh_pvt:
            sv_col = (self.GOOD if s.num_sv >= 8 else
                      self.WARN if s.num_sv >= 5 else self.BAD)
            self.canvas.create_text(cx, ys+176,
                                    text=f'위성 {s.num_sv}기  PDOP={s.p_dop:.1f}',
                                    font=('Helvetica', 13, 'bold'), fill=sv_col)
        else:
            self.canvas.create_text(cx, ys+176,
                                    text='위성 --- 기',
                                    font=('Helvetica', 13), fill=self.MUTED)

        # ── 속도 ──
        fresh_vel = self._fresh(s.vel_stamp, now)
        spd_col = self.CYAN if fresh_vel else self.MUTED
        self.canvas.create_text(cx, ys+205,
                                text=f'속도  {s.speed_mps:.2f} m/s',
                                font=('Courier', 13, 'bold'), fill=spd_col)

        # ── 수치 ──
        self._label_value(lx, ys+245, 'lat',     f'{s.gps_lat:.7f}°')
        self._label_value(lx, ys+268, 'lon',     f'{s.gps_lon:.7f}°')
        self._label_value(lx, ys+291, 'h_acc',   f'{h_acc:.4f} m',  acc_col)
        self._label_value(lx, ys+314, 'speed',   f'{s.speed_mps:.3f} m/s', spd_col)
        self._label_value(lx, ys+337, 'odom_x',  f'{s.odom_x:.3f} m')
        self._label_value(lx, ys+360, 'odom_y',  f'{s.odom_y:.3f} m')

    def _fix_info(self, s):
        if not self._fresh(s.pvt_stamp, 2.0):
            # NavSatFix 기반
            if s.gps_status < 0:
                return 'NO FIX', self.BAD
            if s.gps_status == 0:
                return 'FIX', self.WARN
            return 'FIX+SBAS', self.GOOD
        ft = s.fix_type
        if ft == 0:   return 'NO FIX',       self.BAD
        if ft == 1:   return 'DR',            self.WARN
        if ft == 2:   return '2D FIX',        self.WARN
        if ft == 3:   return '3D FIX',        self.GOOD
        if ft == 4:   return 'GNSS+DR',       self.GOOD
        return 'TIME ONLY', self.MUTED

    def _rtk_info(self, carr_soln):
        if carr_soln == 0: return 'NONE',  self.BAD
        if carr_soln == 1: return 'FLOAT', self.WARN
        if carr_soln == 2: return 'FIXED', self.GOOD
        return '---', self.MUTED

    def _draw_accuracy_gauge(self, cx, cy, bar_w, value, max_val, col):
        c = self.canvas
        half = bar_w // 2
        c.create_rectangle(cx-half, cy-6, cx+half, cy+6,
                           fill='#ddd6c4', outline='')
        ratio = min(1.0, value / max_val)
        fill_w = max(4, int(ratio * bar_w))
        c.create_rectangle(cx-half, cy-5, cx-half+fill_w, cy+5,
                           fill=col, outline='')

    # ── 헤딩 패널 ─────────────────────────────────

    def _draw_head_panel(self, s, now, x, y, w, h):
        fresh_eskf = self._fresh(s.eskf_stamp, now)
        fresh_dual = self._fresh(s.dual_stamp, now)
        self._card(x, y, w, h, '3. Heading (ESKF)')
        self._pill(x+w-130, y+12, 'LIVE' if fresh_eskf else 'STALE',
                   self.GOOD if fresh_eskf else self.BAD)

        cx = x + w//2
        lx = x + 16
        ys = y + 55

        # ── 컴퍼스 ──
        self._draw_compass(cx, ys+90, 80, s.eskf_heading, s.dual_heading,
                           fresh_eskf, fresh_dual)

        # ── ESKF 헤딩 (메인) ──
        eskf_col = self.BLUE if fresh_eskf else self.MUTED
        self.canvas.create_text(cx, ys+195,
                                text=f'ESKF  {s.eskf_heading:+.2f}°',
                                font=('Courier', 16, 'bold'), fill=eskf_col)

        # ESKF 불확도
        if s.eskf_cov > 0:
            sigma_deg = math.degrees(math.sqrt(s.eskf_cov / 15.0 + 1e-10))
            self.canvas.create_text(cx, ys+220,
                                    text=f'σ ≈ ±{sigma_deg:.2f}°',
                                    font=('Helvetica', 12), fill=self.MUTED)

        # ── 듀얼 GPS 헤딩 ──
        dual_col = (self.GOOD if fresh_dual and s.dual_valid else
                    self.WARN if fresh_dual else self.MUTED)
        self.canvas.create_text(cx, ys+255,
                                text=f'Dual GPS  {s.dual_heading:+.2f}°',
                                font=('Courier', 14, 'bold'), fill=dual_col)
        acc_txt = f'±{s.dual_accuracy:.2f}°' if s.dual_accuracy < 90 else '---'
        bl_txt  = f'{s.dual_baseline:.3f}m'  if s.dual_baseline > 0 else '---'
        self.canvas.create_text(cx, ys+277,
                                text=f'정확도 {acc_txt}  베이스라인 {bl_txt}',
                                font=('Helvetica', 11), fill=self.MUTED)

        # 듀얼 GPS 유효성 표시
        valid_col = self.GOOD if s.dual_valid else self.BAD
        self._pill(cx-40, ys+296, 'VALID' if s.dual_valid else 'INVALID',
                   valid_col, size=10)

        # ── IMU 헤딩 ──
        imu_col = self.PURPLE if self._fresh(s.imu_stamp, now) else self.MUTED
        self.canvas.create_text(cx, ys+338,
                                text=f'IMU yaw  {s.imu_yaw_deg:+.2f}°',
                                font=('Courier', 13, 'bold'), fill=imu_col)

        # ── 헤딩 차이 ──
        if fresh_dual and fresh_eskf:
            diff = s.eskf_heading - s.dual_heading
            while diff > 180:  diff -= 360
            while diff < -180: diff += 360
            diff_col = (self.GOOD if abs(diff) < 1.0 else
                        self.WARN if abs(diff) < 3.0 else self.BAD)
            self.canvas.create_text(cx, ys+368,
                                    text=f'ESKF-Dual Δ = {diff:+.2f}°',
                                    font=('Helvetica', 12, 'bold'), fill=diff_col)

        # ── 수치 ──
        self._label_value(lx, ys+405, 'eskf_hdg',  f'{s.eskf_heading:.3f}°',  eskf_col)
        self._label_value(lx, ys+428, 'dual_hdg',  f'{s.dual_heading:.3f}°',  dual_col)
        self._label_value(lx, ys+451, 'dual_acc',  f'{s.dual_accuracy:.3f}°')
        self._label_value(lx, ys+474, 'baseline',  f'{s.dual_baseline:.4f}m')
        self._label_value(lx, ys+497, 'imu_yaw',   f'{s.imu_yaw_deg:.3f}°',   imu_col)

    def _draw_compass(self, cx, cy, r, eskf_hdg, dual_hdg, fresh_eskf, fresh_dual):
        c = self.canvas
        # 컴퍼스 링
        c.create_oval(cx-r, cy-r, cx+r, cy+r,
                      outline='#c8bba0', width=2, fill='#faf5ea')
        # 방위 글자
        for txt, angle in [('N',0),('E',90),('S',180),('W',270)]:
            rad = math.radians(angle - 90)
            tx = cx + (r+14) * math.cos(rad)
            ty = cy + (r+14) * math.sin(rad)
            c.create_text(tx, ty, text=txt,
                          font=('Helvetica', 10, 'bold'), fill=self.MUTED)
        # 눈금
        for deg in range(0, 360, 30):
            rad = math.radians(deg - 90)
            inner = r - 8 if deg % 90 == 0 else r - 5
            c.create_line(cx + inner*math.cos(rad), cy + inner*math.sin(rad),
                          cx + r*math.cos(rad), cy + r*math.sin(rad),
                          fill='#bbb4a0', width=1 if deg % 90 else 2)

        # 듀얼 GPS 헤딩 화살표 (초록)
        if fresh_dual:
            rad = math.radians(dual_hdg - 90)
            c.create_line(cx, cy,
                          cx + (r-12)*math.cos(rad), cy + (r-12)*math.sin(rad),
                          fill=self.GOOD, width=3, arrow=tk.LAST)

        # ESKF 헤딩 화살표 (파랑, 메인)
        if fresh_eskf:
            rad = math.radians(eskf_hdg - 90)
            c.create_line(cx, cy,
                          cx + (r-4)*math.cos(rad), cy + (r-4)*math.sin(rad),
                          fill=self.BLUE, width=4, arrow=tk.LAST)

        # 중심점
        c.create_oval(cx-4, cy-4, cx+4, cy+4, fill=self.TEXT, outline='')
        # 범례
        c.create_line(cx-r, cy+r+14, cx-r+18, cy+r+14,
                      fill=self.BLUE, width=3)
        c.create_text(cx-r+22, cy+r+14, text='ESKF', anchor='w',
                      font=('Helvetica', 9, 'bold'), fill=self.BLUE)
        c.create_line(cx-r+60, cy+r+14, cx-r+78, cy+r+14,
                      fill=self.GOOD, width=3)
        c.create_text(cx-r+82, cy+r+14, text='Dual GPS', anchor='w',
                      font=('Helvetica', 9, 'bold'), fill=self.GOOD)

    # ── Raw 하단 바 ───────────────────────────────

    def _draw_raw_bar(self, s, now, x, y, w, h):
        c = self.canvas
        c.create_rectangle(x, y, x+w, y+h, fill=self.PANEL_H,
                           outline='#cfbea0', width=1)
        lx = x + 18
        ly = y + 22

        # Gyro
        c.create_text(lx, ly, anchor='w',
                      text=(f'Gyro  x={s.gyro_x:+.3f}  '
                            f'y={s.gyro_y:+.3f}  '
                            f'z={s.gyro_z:+.3f}  [rad/s]'),
                      font=('Courier', 12), fill=self.TEXT)
        # Accel
        c.create_text(lx, ly+26, anchor='w',
                      text=(f'Accel x={s.accel_x:+.3f}  '
                            f'y={s.accel_y:+.3f}  '
                            f'z={s.accel_z:+.3f}  [m/s²]'),
                      font=('Courier', 12), fill=self.TEXT)
        # Vel
        c.create_text(lx, ly+52, anchor='w',
                      text=(f'GPS vel  vx={s.vel_x:+.3f}  '
                            f'vy={s.vel_y:+.3f}  '
                            f'speed={s.speed_mps:.3f}  [m/s]'),
                      font=('Courier', 12), fill=self.TEXT)
        # ESKF pos
        c.create_text(lx + 650, ly, anchor='w',
                      text=f'ESKF pos  x={s.odom_x:.3f}  y={s.odom_y:.3f}  [m]',
                      font=('Courier', 12), fill=self.BLUE)
        c.create_text(lx + 650, ly+26, anchor='w',
                      text=(f'ESKF hdg={s.eskf_heading:.2f}°  '
                            f'Dual={s.dual_heading:.2f}°  '
                            f'IMU={s.imu_yaw_deg:.2f}°'),
                      font=('Courier', 12), fill=self.TEXT)
        # Staleness
        imu_ok  = self._fresh(s.imu_stamp,  now)
        gps_ok  = self._fresh(s.gps_stamp,  now)
        eskf_ok = self._fresh(s.eskf_stamp, now)
        dual_ok = self._fresh(s.dual_stamp, now)
        for label, ok, xi in [('IMU', imu_ok, lx+650),
                               ('GPS', gps_ok, lx+730),
                               ('ESKF', eskf_ok, lx+810),
                               ('Dual', dual_ok, lx+900)]:
            self._pill(xi, ly+44, label,
                       self.GOOD if ok else self.BAD, size=9)


# ============================================================
# main
# ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorMonitorNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        SensorMonitorWindow(node).run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
