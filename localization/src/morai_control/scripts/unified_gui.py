#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, json, sys
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
from std_msgs.msg import Float32MultiArray, String
import numpy as np
from collections import deque

MAXLEN = 300  # 15초 (20Hz 기준)

class UnifiedPerformanceGUI:
    def __init__(self, path_file):
        rospy.init_node('unified_performance_gui')

        # ── 상태 변수 ─────────────────────────────────────────
        # GPS/IMU ESKF localization 기준
        self.cur_x      = 0.0
        self.cur_y      = 0.0
        self.cur_yaw    = 0.0   # ROS yaw [rad], 0=East CCW
        self.cur_vel    = 0.0   # [km/h]
        self.last_steer = 0.0   # [deg]
        self.last_vcmd  = 0.0   # [km/h]
        self.solve_time = 0.0
        self.solver_status = "N/A"
        self.start_time = rospy.get_time()

        # ── 진단 버퍼 ─────────────────────────────────────────
        self.t_buf     = deque(maxlen=MAXLEN)
        self.dr_buf    = deque(maxlen=MAXLEN)   # 횡방향 오차 [m]
        self.he_buf    = deque(maxlen=MAXLEN)   # 헤딩 오차 [deg]
        self.steer_buf = deque(maxlen=MAXLEN)   # 조향 명령 [deg]
        self.sol_buf   = deque(maxlen=MAXLEN)   # 솔버 시간 [ms]
        self.vcmd_buf  = deque(maxlen=MAXLEN)   # 속도 명령 [km/h]
        self.vact_buf  = deque(maxlen=MAXLEN)   # 실제 속도 [km/h]
        self.trail_x   = deque(maxlen=MAXLEN)
        self.trail_y   = deque(maxlen=MAXLEN)

        # ── 경로 로드 ─────────────────────────────────────────
        with open(path_file) as f:
            data = json.load(f)
        self.waypoints = np.array([[wp['x'], wp['y']] for wp in data['waypoints']])
        self.wp_h = np.array([wp['heading'] for wp in data['waypoints']])  # ROS yaw [rad]

        # ── 구독 ──────────────────────────────────────────────
        rospy.Subscriber('/localization/ego_status',
                         EgoVehicleStatus, self.ego_callback)
        rospy.Subscriber('/ctrl_cmd_0',      CtrlCmd,           self.cmd_callback)
        rospy.Subscriber('/mpc_performance', Float32MultiArray, self.mpc_perf_callback)
        rospy.Subscriber('/mpc_status',      String,            self.mpc_status_callback)

        # ── 레이아웃 (3행 3열) ───────────────────────────────
        plt.ion()
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('MPC Solver Diagnostics', fontsize=13, fontweight='bold')
        gs = gridspec.GridSpec(3, 3, figure=self.fig, hspace=0.45, wspace=0.35)

        # [열0 전체] 경로 맵
        self.ax_path = self.fig.add_subplot(gs[:, 0])
        self.ax_path.plot(self.waypoints[:, 0], self.waypoints[:, 1],
                          'k--', lw=1, label='Planned Path')
        self.curr_pos_plot, = self.ax_path.plot([], [], 'ro', ms=6, label='Current Pos')
        self.trail_line,    = self.ax_path.plot([], [], 'b-', lw=0.8, alpha=0.5, label='Trail')
        self.ax_path.set_title('Global Path & Tracking')
        self.ax_path.set_aspect('equal')
        self.ax_path.legend(fontsize=8)

        # [0,1] 헤딩 오차 (ESKF 기준)
        self.ax_hdg = self.fig.add_subplot(gs[0, 1])
        self.line_hdg, = self.ax_hdg.plot([], [], 'g-', lw=1, label='Heading error [deg]')
        self.ax_hdg.axhline(0,   color='k', lw=0.5)
        self.ax_hdg.axhline( 10, color='r', lw=0.8, ls='--', label='±10°')
        self.ax_hdg.axhline(-10, color='r', lw=0.8, ls='--')
        self.ax_hdg.set_title('Heading error [deg]')
        self.ax_hdg.set_ylim(-45, 45)
        self.ax_hdg.legend(fontsize=8)

        # [0,2] 횡방향 오차 dr
        self.ax_dr = self.fig.add_subplot(gs[0, 2])
        self.ax_dr.set_title('Lateral error dr [m]')
        self.ax_dr.axhline(0,    color='k', lw=0.5)
        self.ax_dr.axhline( 0.5, color='r', lw=0.8, ls='--', label='±0.5m')
        self.ax_dr.axhline(-0.5, color='r', lw=0.8, ls='--')
        self.line_dr, = self.ax_dr.plot([], [], 'b-', lw=1)
        self.ax_dr.set_ylim(-2, 2)
        self.ax_dr.legend(fontsize=7)

        # [1,1] 조향 명령
        self.ax_steer = self.fig.add_subplot(gs[1, 1])
        self.ax_steer.set_title('Steering cmd [deg]')
        self.ax_steer.axhline(0,   color='k', lw=0.5)
        self.ax_steer.axhline( 35, color='r', lw=0.8, ls='--', label='±35° limit')
        self.ax_steer.axhline(-35, color='r', lw=0.8, ls='--')
        self.line_steer, = self.ax_steer.plot([], [], 'm-', lw=1)
        self.ax_steer.set_ylim(-50, 50)
        self.ax_steer.legend(fontsize=7)

        # [1,2] 솔버 시간
        self.ax_sol = self.fig.add_subplot(gs[1, 2])
        self.ax_sol.set_title('Solver time [ms]')
        self.ax_sol.axhline(50, color='r', lw=0.8, ls='--', label='50ms deadline')
        self.line_sol, = self.ax_sol.plot([], [], 'c-', lw=1)
        self.ax_sol.set_ylim(0, 100)
        self.ax_sol.legend(fontsize=7)

        # [2,1] 속도 비교
        self.ax_vel = self.fig.add_subplot(gs[2, 1])
        self.ax_vel.set_title('Velocity [km/h]')
        self.line_vcmd, = self.ax_vel.plot([], [], 'r-',  lw=1, label='cmd')
        self.line_vact, = self.ax_vel.plot([], [], 'b--', lw=1, label='actual')
        self.ax_vel.set_ylim(0, 40)
        self.ax_vel.legend(fontsize=7)

        # [2,2] 상태 텍스트
        self.ax_info = self.fig.add_subplot(gs[2, 2])
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.05, 0.95, '',
                                           transform=self.ax_info.transAxes,
                                           fontsize=10, fontweight='bold',
                                           family='monospace', va='top')

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.update_plot()
            rate.sleep()

    # ── 콜백 ─────────────────────────────────────────────────
    def ego_callback(self, msg):
        self.cur_x = msg.position.x
        self.cur_y = msg.position.y
        self.cur_yaw = np.radians(msg.heading)
        self.cur_yaw = np.arctan2(np.sin(self.cur_yaw), np.cos(self.cur_yaw))
        # 속도 km/h
        self.cur_vel = np.hypot(msg.velocity.x, msg.velocity.y) * 3.6

    def cmd_callback(self, msg):
        self.last_steer = msg.steering
        self.last_vcmd  = msg.velocity

    def mpc_perf_callback(self, msg):
        if len(msg.data) >= 2:
            self.solve_time = msg.data[1]

    def mpc_status_callback(self, msg):
        self.solver_status = msg.data

    # ── 진단 버퍼 기록 ───────────────────────────────────────
    def _record_diag(self):
        t = rospy.get_time() - self.start_time
        self.t_buf.append(t)

        # 가장 가까운 웨이포인트
        dists = np.hypot(self.waypoints[:, 0] - self.cur_x,
                         self.waypoints[:, 1] - self.cur_y)
        idx = int(np.argmin(dists))
        ni  = min(idx + 1, len(self.waypoints) - 1)

        # 경로 헤딩 (웨이포인트 간 방향)
        wp_yaw = np.arctan2(self.waypoints[ni, 1] - self.waypoints[idx, 1],
                            self.waypoints[ni, 0] - self.waypoints[idx, 0])

        # 횡방향 오차 dr [m]
        dx = self.cur_x - self.waypoints[idx, 0]
        dy = self.cur_y - self.waypoints[idx, 1]
        dr = -np.sin(wp_yaw) * dx + np.cos(wp_yaw) * dy
        self.dr_buf.append(dr)

        # 헤딩 오차 [deg] - 둘 다 ROS yaw 기준
        he = self.cur_yaw - wp_yaw
        he = (he + np.pi) % (2 * np.pi) - np.pi
        self.he_buf.append(np.degrees(he))

        self.steer_buf.append(self.last_steer)
        self.sol_buf.append(self.solve_time)
        self.vcmd_buf.append(self.last_vcmd)
        self.vact_buf.append(self.cur_vel)
        self.trail_x.append(self.cur_x)
        self.trail_y.append(self.cur_y)

    # ── 플롯 갱신 ─────────────────────────────────────────────
    def update_plot(self):
        self._record_diag()
        tarr = list(self.t_buf)
        if not tarr:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            return

        tmin = max(0, tarr[-1] - 15)
        tmax = tarr[-1] + 0.5

        # 경로 맵
        self.curr_pos_plot.set_data(self.cur_x, self.cur_y)
        self.trail_line.set_data(list(self.trail_x), list(self.trail_y))

        # 헤딩 오차
        self.line_hdg.set_data(tarr, list(self.he_buf))
        self.ax_hdg.set_xlim(tmin, tmax)

        # 횡방향 오차
        self.line_dr.set_data(tarr, list(self.dr_buf))
        self.ax_dr.set_xlim(tmin, tmax)

        # 조향
        self.line_steer.set_data(tarr, list(self.steer_buf))
        self.ax_steer.set_xlim(tmin, tmax)

        # 솔버 시간
        self.line_sol.set_data(tarr, list(self.sol_buf))
        self.ax_sol.set_xlim(tmin, tmax)
        if self.sol_buf:
            self.ax_sol.set_ylim(0, max(100, max(self.sol_buf) * 1.2))

        # 속도
        self.line_vcmd.set_data(tarr, list(self.vcmd_buf))
        self.line_vact.set_data(tarr, list(self.vact_buf))
        self.ax_vel.set_xlim(tmin, tmax)

        # 상태 텍스트
        dr_now  = self.dr_buf[-1]    if self.dr_buf    else 0.0
        he_now  = self.he_buf[-1]    if self.he_buf    else 0.0
        st_now  = self.steer_buf[-1] if self.steer_buf else 0.0
        sol_now = self.sol_buf[-1]   if self.sol_buf   else 0.0
        v_now   = self.vact_buf[-1]  if self.vact_buf  else 0.0

        warns = []
        if abs(dr_now) > 0.5: warns.append('!! LARGE dr')
        if abs(he_now) > 15:  warns.append('!! HEADING ERR')
        if abs(st_now) > 34:  warns.append('!! STEER SAT')
        if sol_now     > 50:  warns.append('!! SLOW SOLVER')

        info = (f"=== MPC Status ===\n"
                f"Solver  : {self.solver_status}\n"
                f"dr      : {dr_now:+.3f} m\n"
                f"hdg err : {he_now:+.1f} deg\n"
                f"steer   : {st_now:+.1f} deg\n"
                f"solver  : {sol_now:.1f} ms\n"
                f"vel act : {v_now:.1f} km/h\n"
                + ('\n'.join(warns) if warns else 'OK'))
        self.info_text.set_text(info)
        self.info_text.set_color('red' if warns else 'darkgreen')

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


if __name__ == '__main__':
    path_file = sys.argv[1] if len(sys.argv) > 1 else '/home/david/recorded_path.json'
    try:
        UnifiedPerformanceGUI(path_file)
    except rospy.ROSInterruptException:
        pass
