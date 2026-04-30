#!/usr/bin/env python3
"""
MPC 경로 추종 GUI 뷰어 — 글로벌 패스 + 실제 궤적
mpc_log.json 변경 감지 시 자동 갱신
"""
import sys, os, json, math, time
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.collections import LineCollection
from matplotlib.animation import FuncAnimation

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
PKG_DIR     = os.path.dirname(SCRIPT_DIR)
LOG_PATH    = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PKG_DIR, "logs", "mpc_log.json")
WP_PATH     = sys.argv[2] if len(sys.argv) > 2 else os.path.join(PKG_DIR, "data", "waypoints.json")

time.sleep(2)  # 다른 노드 기동 대기

# ── 웨이포인트 로드 ────────────────────────────────────────────────────
with open(WP_PATH) as f:
    wps   = json.load(f)["waypoints"]
wp_x = np.array([w["x"] for w in wps])
wp_y = np.array([w["y"] for w in wps])

# ── Figure 세팅 ────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(10, 8))
fig.patch.set_facecolor("#1e1e1e")
ax.set_facecolor("#2b2b2b")
ax.tick_params(colors="#aaa")
for sp in ax.spines.values():
    sp.set_color("#555")
ax.set_aspect("equal")
ax.grid(True, alpha=0.2, color="#555")

# 글로벌 패스 (고정)
ax.plot(wp_x, wp_y, "--", color="#5599ff", linewidth=1.5,
        label="Global Path", zorder=2)
ax.scatter(wp_x[0],  wp_y[0],  c="lime",        s=80, zorder=5)
ax.scatter(wp_x[-1], wp_y[-1], c="deepskyblue", s=80, zorder=5)

# 가변 요소
traj_col  = [None]   # LineCollection
recov_sc  = [None]   # scatter
ego_sc    = [None]   # scatter (현재 위치)
title_ref = [None]

ax.legend(loc="upper left", fontsize=8, facecolor="#333", labelcolor="#eee")
plt.tight_layout()

# ── 데이터 로드 & 갱신 ─────────────────────────────────────────────────
last_mtime = [0]

def reload(records):
    if not records:
        return

    x   = np.array([r["x"]   for r in records])
    y   = np.array([r["y"]   for r in records])
    cte = np.array([r["cte"] for r in records])
    mode= np.array([r.get("mode", "NORMAL") for r in records])

    # 궤적 LineCollection
    if traj_col[0] is not None:
        traj_col[0].remove()
    pts  = np.c_[x, y].reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    norm = mcolors.Normalize(0, 1.5)
    lc   = LineCollection(segs, cmap=plt.cm.RdYlGn_r, norm=norm,
                          linewidth=2.2, zorder=3, alpha=0.9)
    lc.set_array(np.abs(cte[:-1]))
    ax.add_collection(lc)
    traj_col[0] = lc

    # RECOV 구간
    if recov_sc[0] is not None:
        recov_sc[0].remove()
    rm = mode == "RECOV"
    if rm.any():
        recov_sc[0] = ax.scatter(x[rm], y[rm], c="orange",
                                 s=12, zorder=4, alpha=0.7)
    else:
        recov_sc[0] = None

    # 현재 차량 위치 (마지막 점)
    if ego_sc[0] is not None:
        ego_sc[0].remove()
    ego_sc[0] = ax.scatter(x[-1], y[-1], c="white", s=100,
                           marker="^", zorder=6)

    # 제목 통계
    cte_abs  = np.abs(cte)
    rmse     = math.sqrt(float(np.mean(cte**2)))
    p30      = float((cte_abs < 0.30).sum()) / len(records) * 100
    n_recov  = int(rm.sum())
    t_end    = records[-1].get("t", 0)
    fig.suptitle(
        f"CTE RMSE={rmse:.3f}m  │  30cm 이내={p30:.1f}%  "
        f"│  RECOV={n_recov}tick  │  t={t_end:.1f}s",
        color="#eee", fontsize=9
    )

    # 뷰 자동 조정 (처음 한 번만)
    if last_mtime[0] == 0:
        ax.autoscale_view()

def update(_frame):
    if not os.path.exists(LOG_PATH):
        return
    mtime = os.path.getmtime(LOG_PATH)
    if mtime == last_mtime[0]:
        return
    last_mtime[0] = mtime
    try:
        with open(LOG_PATH) as f:
            records = json.load(f)["records"]
        reload(records)
    except Exception:
        pass

ani = FuncAnimation(fig, update, interval=2000, cache_frame_data=False)
plt.show()
