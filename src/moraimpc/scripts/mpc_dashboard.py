#!/usr/bin/env python3
"""
MPC 실시간 대시보드 — 전진/후진 명확 구분 + 글로벌 패스 + 추종율 % + 헤딩 오차율 % + 기어 인디케이터
"""
import json
import os
import rospy
import math
import collections
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from std_msgs.msg import Float32MultiArray, String
from morai_msgs.msg import EgoVehicleStatus

# ── 실시간 버퍼 ──────────────────────────────────────────────────────
BUFLEN = 2000  # 최근 ~100초 (20Hz)

t_buf      = collections.deque(maxlen=BUFLEN)
cte_buf    = collections.deque(maxlen=BUFLEN)
hdg_buf    = collections.deque(maxlen=BUFLEN)
vel_buf    = collections.deque(maxlen=BUFLEN)
tgt_v_buf  = collections.deque(maxlen=BUFLEN)

trail_x    = collections.deque(maxlen=4000)
trail_y    = collections.deque(maxlen=4000)

# 누적 통계
total_ticks   = [0]
ticks_10      = [0]
ticks_20      = [0]
ticks_30      = [0]
ticks_50      = [0]
ticks_hdg_ok  = [0]   # 헤딩 오차 임계 이내 카운터
cte_sq        = [0.0]
hdg_sq        = [0.0]
recov_n       = [0]
gear_switch_n = [0]
nmpc_n        = [0]   # NMPC 활성 틱 수
cur_mode      = ["—"]
cur_gear_str  = ["D"]
cur_ctrl      = ["LTV"]   # LTV / NMPC / RECOV
prev_gear     = ["D"]
t0            = [None]

# 차량 위치 (글로벌 맵 갱신용)
ego_x   = [None]
ego_y   = [None]
ego_yaw = [0.0]

# ── ROS 초기화 ────────────────────────────────────────────────────────
rospy.init_node("mpc_dashboard", anonymous=True)

# 헤딩 오차 임계값 (도) — 기본 5°
HDG_PCT_THRESH = float(rospy.get_param("~hdg_pct_thresh_deg", 5.0))

# 글로벌 패스 로드 (D/R 분리)
path_file = rospy.get_param("~path_file", "")
if not path_file:
    # fallback: path_follower_node 파라미터에서 가져오기
    path_file = rospy.get_param("/path_follower_node/path_file", "")

wp_d_x, wp_d_y, wp_r_x, wp_r_y = [], [], [], []
if path_file and os.path.isfile(path_file):
    try:
        with open(path_file) as f:
            wps = json.load(f).get("waypoints", [])
        for w in wps:
            g = w.get("gear", "D")
            if g == "R":
                wp_r_x.append(w["x"]); wp_r_y.append(w["y"])
            else:
                wp_d_x.append(w["x"]); wp_d_y.append(w["y"])
        rospy.loginfo("[dashboard] 패스 로드: %d D + %d R 점", len(wp_d_x), len(wp_r_x))
    except Exception as e:
        rospy.logwarn("[dashboard] 패스 로드 실패: %s", e)
else:
    rospy.logwarn("[dashboard] path_file 없음 — 글로벌 패스 표시 비활성")


def perf_cb(msg):
    if len(msg.data) < 7:
        return
    now = rospy.Time.now().to_sec()
    if t0[0] is None:
        t0[0] = now
    elapsed = now - t0[0]

    cte  = float(msg.data[2])
    hdg  = float(msg.data[3])
    v    = float(msg.data[4])
    tgt  = float(msg.data[6])

    t_buf.append(elapsed)
    cte_buf.append(cte)
    hdg_buf.append(hdg)
    vel_buf.append(v)
    tgt_v_buf.append(tgt)

    cte_abs = abs(cte)
    total_ticks[0] += 1
    cte_sq[0] += cte ** 2
    hdg_sq[0] += hdg ** 2
    if cte_abs < 0.10: ticks_10[0] += 1
    if cte_abs < 0.20: ticks_20[0] += 1
    if cte_abs < 0.30: ticks_30[0] += 1
    if cte_abs < 0.50: ticks_50[0] += 1
    if abs(hdg) < HDG_PCT_THRESH: ticks_hdg_ok[0] += 1


def status_cb(msg):
    cur_mode[0] = msg.data
    # 컨트롤러 종류: NMPC_*, RECOV*, NORMAL*
    if msg.data.startswith("NMPC"):
        cur_ctrl[0] = "NMPC"
        nmpc_n[0] += 1
    elif msg.data.startswith("RECOV"):
        cur_ctrl[0] = "RECOV"
        recov_n[0] += 1
    else:
        cur_ctrl[0] = "LTV"
    # 기어 추출 — *_R 접미사 (NMPC_R, NORMAL_R, RECOV_R 모두 R)
    g = "R" if msg.data.endswith("_R") else "D"
    if g != prev_gear[0]:
        gear_switch_n[0] += 1
        prev_gear[0] = g
    cur_gear_str[0] = g


def ego_cb(msg):
    ego_x[0] = msg.position.x
    ego_y[0] = msg.position.y
    # MORAI heading: deg → rad
    ego_yaw[0] = msg.heading * math.pi / 180.0
    trail_x.append(ego_x[0])
    trail_y.append(ego_y[0])


rospy.Subscriber("/mpc_performance", Float32MultiArray, perf_cb)
rospy.Subscriber("/mpc_status",      String,            status_cb)
rospy.Subscriber("/Ego_topic",       EgoVehicleStatus,  ego_cb)

# ── 대시보드 레이아웃 ─────────────────────────────────────────────────
fig = plt.figure(figsize=(13, 11), facecolor="#1a1a2e")
fig.canvas.manager.set_window_title("MPC Dashboard")

# 5행 레이아웃: [상단 게이지] [맵 뷰(크게)] [CTE] [HDG] [속도]
gs = GridSpec(5, 2, figure=fig, hspace=0.55, wspace=0.3,
              left=0.06, right=0.97, top=0.94, bottom=0.05,
              height_ratios=[1.2, 2.4, 1, 1, 1])

# 상단 좌측: CTE % + HDG %
top_left  = gs[0, 0].subgridspec(1, 2, wspace=0.15)
ax_pct     = fig.add_subplot(top_left[0, 0]); ax_pct.set_facecolor("#1a1a2e"); ax_pct.axis("off")
ax_pct_hdg = fig.add_subplot(top_left[0, 1]); ax_pct_hdg.set_facecolor("#1a1a2e"); ax_pct_hdg.axis("off")
txt_pct = ax_pct.text(0.5, 0.55, "—%", transform=ax_pct.transAxes,
                       fontsize=64, fontweight="bold", color="white",
                       ha="center", va="center", family="monospace")
ax_pct.text(0.5, 0.05, "경로일치율 (≤20cm)",
            transform=ax_pct.transAxes, fontsize=12, color="#aaaaaa",
            ha="center", va="bottom")
txt_pct_hdg = ax_pct_hdg.text(0.5, 0.55, "—%", transform=ax_pct_hdg.transAxes,
                               fontsize=64, fontweight="bold", color="white",
                               ha="center", va="center", family="monospace")
ax_pct_hdg.text(0.5, 0.05, f"헤딩일치율 (≤{HDG_PCT_THRESH:.0f}°)",
                transform=ax_pct_hdg.transAxes, fontsize=12, color="#aaaaaa",
                ha="center", va="bottom")

# 상단 우측: 통계 + 기어 인디케이터
top_right = gs[0, 1].subgridspec(1, 2, wspace=0.1, width_ratios=[2, 1])
ax_stats = fig.add_subplot(top_right[0, 0]); ax_stats.set_facecolor("#16213e"); ax_stats.axis("off")
ax_gear  = fig.add_subplot(top_right[0, 1]); ax_gear.axis("off")
txt_stats = ax_stats.text(0.05, 0.95, "", transform=ax_stats.transAxes,
                           fontsize=11, color="white", va="top",
                           family="monospace", linespacing=1.55)
gear_bg = ax_gear.add_patch(plt.Rectangle((0.05, 0.05), 0.9, 0.9,
                                           transform=ax_gear.transAxes,
                                           facecolor="#54a0ff", alpha=0.85))
txt_gear = ax_gear.text(0.5, 0.55, "D", transform=ax_gear.transAxes,
                         fontsize=72, fontweight="bold", color="white",
                         ha="center", va="center", family="monospace")
txt_gear_label = ax_gear.text(0.5, 0.05, "기어 / LTV", transform=ax_gear.transAxes,
              fontsize=11, color="white", ha="center", va="bottom")

# 1행: 글로벌 맵 (D 파랑 / R 빨강 / 차량 화살표 / 트레일)
ax_map = fig.add_subplot(gs[1, :])
ax_map.set_facecolor("#0f3460")
ax_map.set_aspect("equal")
ax_map.tick_params(colors="white")
ax_map.grid(True, alpha=0.2)
ax_map.set_title("Global Path & Vehicle (D=blue / R=red)", color="white", fontsize=11)
if wp_d_x: ax_map.plot(wp_d_x, wp_d_y, ".", color="#54a0ff", markersize=2.5, label=f"D ({len(wp_d_x)})")
if wp_r_x: ax_map.plot(wp_r_x, wp_r_y, ".", color="#ff6b6b", markersize=2.5, label=f"R ({len(wp_r_x)})")
line_trail, = ax_map.plot([], [], "-", color="#feca57", lw=1.2, alpha=0.65, label="vehicle trail")
veh_marker, = ax_map.plot([], [], "o", color="white", markersize=10, mec="black", mew=1.0, label="ego")
veh_arrow = ax_map.annotate("", xy=(0, 0), xytext=(0, 0),
                             arrowprops=dict(arrowstyle="->", color="white", lw=2.2, mutation_scale=15))
leg_map = ax_map.legend(loc="upper right", fontsize=8, facecolor="#16213e", edgecolor="gray")
for tt in leg_map.get_texts(): tt.set_color("white")

# 2행: CTE 시계열
ax_cte = fig.add_subplot(gs[2, :])
ax_cte.set_facecolor("#0f3460"); ax_cte.set_ylabel("CTE [m]", color="white", fontsize=11)
ax_cte.tick_params(colors="white"); ax_cte.grid(True, alpha=0.2)
ax_cte.axhline(0, color="white", lw=0.5, ls="--")
ax_cte.axhline(0.20, color="yellow", lw=0.8, ls="--", alpha=0.5)
ax_cte.axhline(-0.20, color="yellow", lw=0.8, ls="--", alpha=0.5)
line_cte, = ax_cte.plot([], [], color="#00ff88", lw=1.2)
fill_cte = None

# 3행: HDG 시계열
ax_hdg = fig.add_subplot(gs[3, :])
ax_hdg.set_facecolor("#0f3460"); ax_hdg.set_ylabel("HDG err [°]", color="white", fontsize=11)
ax_hdg.tick_params(colors="white"); ax_hdg.grid(True, alpha=0.2)
ax_hdg.axhline(0, color="white", lw=0.5, ls="--")
ax_hdg.axhline(HDG_PCT_THRESH, color="yellow", lw=0.8, ls="--", alpha=0.5)
ax_hdg.axhline(-HDG_PCT_THRESH, color="yellow", lw=0.8, ls="--", alpha=0.5)
line_hdg, = ax_hdg.plot([], [], color="#ff9f43", lw=1.2)

# 4행: 속도
ax_vel = fig.add_subplot(gs[4, :])
ax_vel.set_facecolor("#0f3460"); ax_vel.set_ylabel("Speed [km/h]", color="white", fontsize=11)
ax_vel.set_xlabel("Time [s]", color="white", fontsize=11)
ax_vel.tick_params(colors="white"); ax_vel.grid(True, alpha=0.2)
line_vel, = ax_vel.plot([], [], color="#54a0ff", lw=1.2, label="actual")
line_tgt, = ax_vel.plot([], [], color="#ff6b6b", lw=1.0, ls="--", label="target")
leg_v = ax_vel.legend(loc="upper right", fontsize=9, facecolor="#16213e", edgecolor="gray")
for tt in leg_v.get_texts(): tt.set_color("white")

fig.suptitle("MPC Real-time Dashboard — D/R Aware", fontsize=15, color="white", fontweight="bold")


def _color_for_pct(pct):
    if pct >= 90: return "#00ff88"
    if pct >= 70: return "#feca57"
    return "#ff6b6b"


def update(frame):
    global fill_cte
    n = total_ticks[0]

    # 퍼센트 표시 (n=0이어도 기어 인디케이터는 갱신)
    if n > 0:
        cte_pct = ticks_20[0] / n * 100
        hdg_pct = ticks_hdg_ok[0] / n * 100
        txt_pct.set_text(f"{cte_pct:.0f}%");     txt_pct.set_color(_color_for_pct(cte_pct))
        txt_pct_hdg.set_text(f"{hdg_pct:.0f}%"); txt_pct_hdg.set_color(_color_for_pct(hdg_pct))

        cte_rmse = math.sqrt(cte_sq[0] / n)
        hdg_rmse = math.sqrt(hdg_sq[0] / n)
        elapsed  = t_buf[-1] if len(t_buf) > 0 else 0
        txt_stats.set_text(
            f"10cm 이내 : {ticks_10[0]/n*100:5.1f}%\n"
            f"20cm 이내 : {ticks_20[0]/n*100:5.1f}%\n"
            f"30cm 이내 : {ticks_30[0]/n*100:5.1f}%\n"
            f"50cm 이내 : {ticks_50[0]/n*100:5.1f}%\n"
            f"≤{HDG_PCT_THRESH:.0f}° 이내 : {hdg_pct:5.1f}%\n"
            f"CTE RMSE  : {cte_rmse:.3f}m\n"
            f"HDG RMSE  : {hdg_rmse:.1f}°\n"
            f"Controller: {cur_ctrl[0]}\n"
            f"Mode      : {cur_mode[0]}\n"
            f"기어 전환 : {gear_switch_n[0]}회\n"
            f"NMPC      : {nmpc_n[0]}틱\n"
            f"RECOV     : {recov_n[0]}회\n"
            f"Time      : {elapsed:.0f}s"
        )

    # 기어 인디케이터 (D=blue, R=red) + 컨트롤러 표시 (NMPC=보라 테두리)
    if cur_gear_str[0] == "R":
        txt_gear.set_text("R"); gear_bg.set_facecolor("#ff6b6b")
    else:
        txt_gear.set_text("D"); gear_bg.set_facecolor("#54a0ff")
    ctrl = cur_ctrl[0]
    txt_gear_label.set_text(f"기어 / {ctrl}")
    if ctrl == "NMPC":
        gear_bg.set_edgecolor("#a55eea"); gear_bg.set_linewidth(4)
    elif ctrl == "RECOV":
        gear_bg.set_edgecolor("#ff9f43"); gear_bg.set_linewidth(4)
    else:
        gear_bg.set_edgecolor("none"); gear_bg.set_linewidth(0)

    if len(t_buf) >= 2:
        t_arr   = np.array(t_buf)
        cte_arr = np.array(cte_buf)
        hdg_arr = np.array(hdg_buf)
        vel_arr = np.array(vel_buf)
        tgt_arr = np.array(tgt_v_buf)

        # CTE
        line_cte.set_data(t_arr, cte_arr)
        ax_cte.set_xlim(max(0, t_arr[-1] - 60), t_arr[-1] + 1)
        cte_lim = max(0.3, np.max(np.abs(cte_arr[-min(len(cte_arr), 1200):])) * 1.3)
        ax_cte.set_ylim(-cte_lim, cte_lim)
        if fill_cte is not None: fill_cte.remove()
        fill_cte = ax_cte.fill_between(t_arr, cte_arr, 0, alpha=0.15, color="#00ff88")

        # HDG
        line_hdg.set_data(t_arr, hdg_arr)
        ax_hdg.set_xlim(max(0, t_arr[-1] - 60), t_arr[-1] + 1)
        hdg_lim = max(5, np.max(np.abs(hdg_arr[-min(len(hdg_arr), 1200):])) * 1.3)
        ax_hdg.set_ylim(-hdg_lim, hdg_lim)

        # Speed
        line_vel.set_data(t_arr, vel_arr); line_tgt.set_data(t_arr, tgt_arr)
        ax_vel.set_xlim(max(0, t_arr[-1] - 60), t_arr[-1] + 1)
        ax_vel.set_ylim(0, max(25, np.max(vel_arr[-min(len(vel_arr), 1200):]) * 1.2))

    # 글로벌 맵: 차량 트레일 + 화살표
    if ego_x[0] is not None:
        line_trail.set_data(list(trail_x), list(trail_y))
        veh_marker.set_data([ego_x[0]], [ego_y[0]])
        L = 3.0
        veh_arrow.xy = (ego_x[0] + L * math.cos(ego_yaw[0]),
                        ego_y[0] + L * math.sin(ego_yaw[0]))
        veh_arrow.set_position((ego_x[0], ego_y[0]))
        # 차량 중심으로 자동 윈도우
        ax_map.set_xlim(ego_x[0] - 30, ego_x[0] + 30)
        ax_map.set_ylim(ego_y[0] - 30, ego_y[0] + 30)


ani = FuncAnimation(fig, update, interval=200, cache_frame_data=False)

# 창 닫기 → ROS 종료 트리거
fig.canvas.mpl_connect("close_event", lambda _e: rospy.signal_shutdown("dashboard closed"))
# ROS 종료 → 창 닫기 (콜백은 보조용, 주된 종료는 폴링 루프가 담당)
rospy.on_shutdown(lambda: plt.close("all"))

# 폴링 루프: rospy.is_shutdown() 체크하면서 GUI 이벤트 처리
plt.ion()
plt.show(block=False)
try:
    while not rospy.is_shutdown():
        try:
            plt.pause(0.1)
        except Exception:
            break
        # 모든 figure 닫혔으면 종료
        if not plt.get_fignums():
            break
finally:
    plt.close("all")
