#!/usr/bin/env python3
"""
MPC 실시간 대시보드 — 별도 창으로 경로일치율/CTE/HDG/속도 실시간 표시
"""
import rospy
import math
import collections
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float32MultiArray, String

# ── 실시간 버퍼 ──────────────────────────────────────────────────────
BUFLEN = 2000  # 최근 ~100초 (20Hz)

t_buf      = collections.deque(maxlen=BUFLEN)
cte_buf    = collections.deque(maxlen=BUFLEN)
hdg_buf    = collections.deque(maxlen=BUFLEN)
vel_buf    = collections.deque(maxlen=BUFLEN)
tgt_v_buf  = collections.deque(maxlen=BUFLEN)

# 누적 통계
total_ticks = [0]
ticks_10  = [0]
ticks_20  = [0]
ticks_30  = [0]
ticks_50  = [0]
cte_sq    = [0.0]
hdg_sq    = [0.0]
recov_n   = [0]
cur_mode  = ["—"]
t0        = [None]


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


def status_cb(msg):
    cur_mode[0] = msg.data
    if msg.data == "RECOV":
        recov_n[0] += 1


# ── ROS 초기화 ────────────────────────────────────────────────────────
rospy.init_node("mpc_dashboard", anonymous=True)
rospy.Subscriber("/mpc_performance", Float32MultiArray, perf_cb)
rospy.Subscriber("/mpc_status",      String,            status_cb)

# ── 대시보드 레이아웃 ─────────────────────────────────────────────────
fig = plt.figure(figsize=(12, 9), facecolor="#1a1a2e")
fig.canvas.manager.set_window_title("MPC Dashboard")

# 4행: [경로일치율 게이지] [CTE] [HDG] [속도]
from matplotlib.gridspec import GridSpec
gs = GridSpec(4, 2, figure=fig, hspace=0.45, wspace=0.3,
              left=0.08, right=0.97, top=0.92, bottom=0.06,
              height_ratios=[1.2, 1, 1, 1])

# (0,0) 큰 퍼센트 숫자
ax_pct = fig.add_subplot(gs[0, 0])
ax_pct.set_facecolor("#1a1a2e")
ax_pct.axis("off")
txt_pct = ax_pct.text(0.5, 0.55, "—%", transform=ax_pct.transAxes,
                       fontsize=72, fontweight="bold", color="white",
                       ha="center", va="center", family="monospace")
txt_pct_label = ax_pct.text(0.5, 0.05, "경로일치율 (20cm 이내)",
                             transform=ax_pct.transAxes,
                             fontsize=13, color="#aaaaaa",
                             ha="center", va="bottom")

# (0,1) 통계 텍스트
ax_stats = fig.add_subplot(gs[0, 1])
ax_stats.set_facecolor("#16213e")
ax_stats.axis("off")
txt_stats = ax_stats.text(0.08, 0.92, "", transform=ax_stats.transAxes,
                           fontsize=13, color="white", va="top",
                           family="monospace", linespacing=1.6)

# (1,:) CTE 시계열
ax_cte = fig.add_subplot(gs[1, :])
ax_cte.set_facecolor("#0f3460")
ax_cte.set_ylabel("CTE [m]", color="white", fontsize=11)
ax_cte.tick_params(colors="white")
ax_cte.grid(True, alpha=0.2)
ax_cte.axhline(0, color="white", lw=0.5, ls="--")
ax_cte.axhline(0.20, color="yellow", lw=0.8, ls="--", alpha=0.5)
ax_cte.axhline(-0.20, color="yellow", lw=0.8, ls="--", alpha=0.5)
line_cte, = ax_cte.plot([], [], color="#00ff88", lw=1.2)
fill_cte = None

# (2,:) HDG 시계열
ax_hdg = fig.add_subplot(gs[2, :])
ax_hdg.set_facecolor("#0f3460")
ax_hdg.set_ylabel("HDG err [°]", color="white", fontsize=11)
ax_hdg.tick_params(colors="white")
ax_hdg.grid(True, alpha=0.2)
ax_hdg.axhline(0, color="white", lw=0.5, ls="--")
line_hdg, = ax_hdg.plot([], [], color="#ff9f43", lw=1.2)

# (3,:) 속도
ax_vel = fig.add_subplot(gs[3, :])
ax_vel.set_facecolor("#0f3460")
ax_vel.set_ylabel("Speed [km/h]", color="white", fontsize=11)
ax_vel.set_xlabel("Time [s]", color="white", fontsize=11)
ax_vel.tick_params(colors="white")
ax_vel.grid(True, alpha=0.2)
line_vel, = ax_vel.plot([], [], color="#54a0ff", lw=1.2, label="actual")
line_tgt, = ax_vel.plot([], [], color="#ff6b6b", lw=1.0, ls="--", label="target")
leg = ax_vel.legend(loc="upper right", fontsize=9, facecolor="#16213e",
                    edgecolor="gray")
for text in leg.get_texts():
    text.set_color("white")

# Title
fig.suptitle("MPC Real-time Dashboard", fontsize=16, color="white", fontweight="bold")


def update(frame):
    global fill_cte

    if len(t_buf) < 2:
        return

    t_arr   = np.array(t_buf)
    cte_arr = np.array(cte_buf)
    hdg_arr = np.array(hdg_buf)
    vel_arr = np.array(vel_buf)
    tgt_arr = np.array(tgt_v_buf)

    n = total_ticks[0]

    # 퍼센트 표시
    if n > 0:
        pct = ticks_20[0] / n * 100
        if pct >= 90:
            color = "#00ff88"
        elif pct >= 70:
            color = "#feca57"
        else:
            color = "#ff6b6b"
        txt_pct.set_text(f"{pct:.0f}%")
        txt_pct.set_color(color)

        cte_rmse = math.sqrt(cte_sq[0] / n)
        hdg_rmse = math.sqrt(hdg_sq[0] / n)
        elapsed = t_arr[-1] if len(t_arr) > 0 else 0

        txt_stats.set_text(
            f"10cm 이내: {ticks_10[0]/n*100:5.1f}%\n"
            f"20cm 이내: {ticks_20[0]/n*100:5.1f}%\n"
            f"30cm 이내: {ticks_30[0]/n*100:5.1f}%\n"
            f"50cm 이내: {ticks_50[0]/n*100:5.1f}%\n"
            f"CTE RMSE : {cte_rmse:.3f}m\n"
            f"HDG RMSE : {hdg_rmse:.1f}°\n"
            f"Mode     : {cur_mode[0]}\n"
            f"RECOV    : {recov_n[0]}\n"
            f"Time     : {elapsed:.0f}s"
        )

    # CTE
    line_cte.set_data(t_arr, cte_arr)
    ax_cte.set_xlim(max(0, t_arr[-1] - 60), t_arr[-1] + 1)
    cte_lim = max(0.3, np.max(np.abs(cte_arr[-min(len(cte_arr), 1200):])) * 1.3)
    ax_cte.set_ylim(-cte_lim, cte_lim)

    if fill_cte is not None:
        fill_cte.remove()
    fill_cte = ax_cte.fill_between(t_arr, cte_arr, 0, alpha=0.15, color="#00ff88")

    # HDG
    line_hdg.set_data(t_arr, hdg_arr)
    ax_hdg.set_xlim(max(0, t_arr[-1] - 60), t_arr[-1] + 1)
    hdg_lim = max(5, np.max(np.abs(hdg_arr[-min(len(hdg_arr), 1200):])) * 1.3)
    ax_hdg.set_ylim(-hdg_lim, hdg_lim)

    # Speed
    line_vel.set_data(t_arr, vel_arr)
    line_tgt.set_data(t_arr, tgt_arr)
    ax_vel.set_xlim(max(0, t_arr[-1] - 60), t_arr[-1] + 1)
    ax_vel.set_ylim(0, max(25, np.max(vel_arr[-min(len(vel_arr), 1200):]) * 1.2))


ani = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
plt.show()
