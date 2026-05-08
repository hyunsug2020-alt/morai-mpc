#!/usr/bin/env python3
"""
MPC driving log analysis script.
Reads mpc_log.json + waypoints.json and saves a PNG visualization.
Output: <log_dir>/mpc_analysis.png

Usage:
  python3 analyze_mpc.py [mpc_log.json] [waypoints.json]
Defaults (no args):
  logs/mpc_log.json  and  data/waypoints.json  relative to package root
"""
import sys
import os
import json
import math
import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
    from matplotlib.gridspec import GridSpec
except ImportError:
    print("matplotlib not found -- pip install matplotlib")
    sys.exit(1)

# ── path resolution ────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR    = os.path.dirname(SCRIPT_DIR)

DEFAULT_LOG = os.path.join(PKG_DIR, "logs", "mpc_log.json")
DEFAULT_WP  = os.path.join(PKG_DIR, "data", "waypoints.json")

log_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG
wp_path  = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_WP
out_path = os.path.join(os.path.dirname(os.path.abspath(log_path)), "mpc_analysis.png")

# ── load data ─────────────────────────────────────────────────────────
print(f"[analyze] log  : {log_path}")
print(f"[analyze] path : {wp_path}")

with open(log_path) as f:
    log_data = json.load(f)
with open(wp_path) as f:
    wp_data = json.load(f)

records = log_data["records"]
wps     = wp_data["waypoints"]

# ── extract arrays ─────────────────────────────────────────────────────
t        = np.array([r["t"]           for r in records])
x        = np.array([r["x"]           for r in records])
y        = np.array([r["y"]           for r in records])
cte      = np.array([r["cte"]         for r in records])
hdg      = np.array([r["hdg_err_deg"] for r in records])
steer    = np.array([r["steer_cmd"]   for r in records])
v        = np.array([r["v_kmh"]       for r in records])
mode     = np.array([r.get("mode", "NORMAL") for r in records])
kappa    = np.array([r.get("current_kappa", float("nan")) for r in records])
kreset   = np.array([r.get("kappa_reset",   False)        for r in records], dtype=bool)
solve_ms = np.array([r.get("solve_ms",       float("nan")) for r in records])

# New fields (graceful fallback for older logs)
target_vel  = np.array([r.get("target_vel",     float("nan")) for r in records])
actual_vel  = np.array([r.get("actual_vel",      float("nan")) for r in records])
vel_error   = np.array([r.get("vel_error",       float("nan")) for r in records])
lat_accel   = np.array([r.get("lateral_accel",   float("nan")) for r in records])
path_curv   = np.array([r.get("path_curvature",  float("nan")) for r in records])
pred_cte    = np.array([r.get("predicted_cte",   float("nan")) for r in records])
yaw_rate    = np.array([r.get("yaw_rate",        float("nan")) for r in records])

wp_x = np.array([w["x"] for w in wps])
wp_y = np.array([w["y"] for w in wps])

has_new_fields = np.isfinite(target_vel).sum() > len(records) * 0.5

# ── figure layout ──────────────────────────────────────────────────────
nrows = 5 if has_new_fields else 3
fig = plt.figure(figsize=(18, 5 * nrows))
gs  = GridSpec(nrows, 2, figure=fig, hspace=0.45, wspace=0.35)

# ─ (1) trajectory vs planned path, coloured by |CTE| ──────────────────
ax1 = fig.add_subplot(gs[0, :])
ax1.plot(wp_x, wp_y, "k--", linewidth=1.2, label="Planned path", zorder=2)

cte_abs = np.abs(cte)
cte_max = max(cte_abs.max(), 1e-3)
norm    = mcolors.Normalize(vmin=0.0, vmax=min(cte_max, 2.0))
cmap    = plt.cm.RdYlGn_r

for i in range(len(x) - 1):
    c = cmap(norm(cte_abs[i]))
    ax1.plot(x[i:i+2], y[i:i+2], color=c, linewidth=2.0, zorder=3)

recov_mask = mode == "RECOV"
stop_mask  = mode == "STOP"
ax1.scatter(x[recov_mask], y[recov_mask], c="orange", s=20, zorder=4, label="RECOV")
ax1.scatter(x[stop_mask],  y[stop_mask],  c="blue",   s=20, zorder=4, label="STOP")
ax1.scatter(x[0],  y[0],  c="lime", s=80, zorder=5, label="Start")
ax1.scatter(x[-1], y[-1], c="red",  s=80, zorder=5, label="End")

sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])
plt.colorbar(sm, ax=ax1, label="|CTE| [m]", pad=0.01)

ax1.set_aspect("equal")
ax1.set_title(f"Actual Trajectory vs Planned Path  ({len(records)} ticks, {t[-1]:.1f}s)")
ax1.set_xlabel("X [m]"); ax1.set_ylabel("Y [m]")
ax1.legend(loc="upper left", fontsize=8)
ax1.grid(True, alpha=0.3)

# ─ (2) CTE time series ─────────────────────────────────────────────────
ax2 = fig.add_subplot(gs[1, 0])
ax2.plot(t, cte, linewidth=1.2, label="CTE [m]")
ax2.axhline(0, color="k", linewidth=0.6, linestyle="--")
ax2.fill_between(t, cte, 0, alpha=0.18)
ax2.set_xlabel("Time [s]"); ax2.set_ylabel("CTE [m]")
ax2.set_title("Cross-Track Error (CTE)")
ax2.legend(); ax2.grid(True, alpha=0.3)

# ─ (3) heading error time series ──────────────────────────────────────
ax3 = fig.add_subplot(gs[1, 1])
ax3.plot(t, hdg, linewidth=1.2, color="darkorange", label="hErr [deg]")
ax3.axhline(0, color="k", linewidth=0.6, linestyle="--")
ax3.set_xlabel("Time [s]"); ax3.set_ylabel("hErr [deg]")
ax3.set_title("Heading Error")
ax3.legend(); ax3.grid(True, alpha=0.3)

# ─ (4) steering + kappa ────────────────────────────────────────────────
ax4 = fig.add_subplot(gs[2, 0])
ax4.plot(t, steer, linewidth=1.2, label="steer_cmd [deg]")
ax4_r = ax4.twinx()
ax4_r.plot(t, kappa, linewidth=1.0, color="green", alpha=0.7,
           linestyle="--", label="kappa [rad/m]")
ax4_r.set_ylabel("kappa [rad/m]", color="green")

if kreset.any():
    for i in range(len(t)):
        if kreset[i]:
            ax4.axvspan(t[i], t[i] + 0.06, alpha=0.2, color="red")

ax4.set_xlabel("Time [s]"); ax4.set_ylabel("steer [deg]")
ax4.set_title("Steering + Curvature  (red = kappa_reset active)")
lines1, labels1 = ax4.get_legend_handles_labels()
lines2, labels2 = ax4_r.get_legend_handles_labels()
ax4.legend(lines1 + lines2, labels1 + labels2, fontsize=8)
ax4.grid(True, alpha=0.3)

# ─ (5) speed + solve_ms ────────────────────────────────────────────────
ax5 = fig.add_subplot(gs[2, 1])
ax5.plot(t, v, linewidth=1.2, label="v [km/h]")
ax5_r = ax5.twinx()
valid_solve = np.isfinite(solve_ms)
ax5_r.scatter(t[valid_solve], solve_ms[valid_solve], s=3,
              color="gray", alpha=0.5, label="solve [ms]")
ax5_r.set_ylabel("OSQP solve [ms]", color="gray")
ax5.set_xlabel("Time [s]"); ax5.set_ylabel("speed [km/h]")
ax5.set_title("Speed Profile + OSQP Solve Time")
lines1, labels1 = ax5.get_legend_handles_labels()
lines2, labels2 = ax5_r.get_legend_handles_labels()
ax5.legend(lines1 + lines2, labels1 + labels2, fontsize=8)
ax5.grid(True, alpha=0.3)

# ── NEW PLOTS (only if new fields available) ──────────────────────────
if has_new_fields:
    # ─ (6) Velocity tracking: target vs actual ────────────────────────
    ax6 = fig.add_subplot(gs[3, 0])
    valid_tv = np.isfinite(target_vel) & np.isfinite(actual_vel)
    ax6.plot(t[valid_tv], target_vel[valid_tv], linewidth=1.2, label="Target [km/h]", color="blue")
    ax6.plot(t[valid_tv], actual_vel[valid_tv], linewidth=1.2, label="Actual [km/h]", color="red", alpha=0.8)
    ax6.fill_between(t[valid_tv], target_vel[valid_tv], actual_vel[valid_tv], alpha=0.15, color="red")
    ax6.set_xlabel("Time [s]"); ax6.set_ylabel("Velocity [km/h]")
    ax6.set_title("Velocity Tracking (Target vs Actual)")
    ax6.legend(); ax6.grid(True, alpha=0.3)

    # ─ (7) Lateral acceleration ───────────────────────────────────────
    ax7 = fig.add_subplot(gs[3, 1])
    valid_la = np.isfinite(lat_accel)
    ax7.plot(t[valid_la], lat_accel[valid_la], linewidth=1.0, color="purple", label="lat_accel [m/s²]")
    ax7.axhline(3.0, color="red", linewidth=0.8, linestyle="--", label="comfort limit")
    ax7.set_xlabel("Time [s]"); ax7.set_ylabel("Lateral Accel [m/s²]")
    ax7.set_title("Lateral Acceleration")
    ax7.legend(); ax7.grid(True, alpha=0.3)

    # ─ (8) Curvature vs CTE scatter ──────────────────────────────────
    ax8 = fig.add_subplot(gs[4, 0])
    valid_kc = np.isfinite(path_curv)
    ax8.scatter(np.abs(path_curv[valid_kc]), np.abs(cte[valid_kc]), s=4, alpha=0.4, color="teal")
    ax8.set_xlabel("|Path Curvature| [rad/m]"); ax8.set_ylabel("|CTE| [m]")
    ax8.set_title("Curvature vs CTE (scatter)")
    ax8.grid(True, alpha=0.3)

    # ─ (9) Predicted CTE vs Actual CTE ───────────────────────────────
    ax9 = fig.add_subplot(gs[4, 1])
    valid_pc = np.isfinite(pred_cte)
    if valid_pc.sum() > 10:
        ax9.scatter(cte[valid_pc], pred_cte[valid_pc], s=4, alpha=0.4, color="darkorange")
        lims = [min(cte[valid_pc].min(), pred_cte[valid_pc].min()),
                max(cte[valid_pc].max(), pred_cte[valid_pc].max())]
        ax9.plot(lims, lims, "k--", linewidth=0.8, label="ideal")
        ax9.set_xlabel("Actual CTE [m]"); ax9.set_ylabel("Predicted CTE [m]")
        ax9.set_title("Predicted vs Actual CTE")
        ax9.legend(); ax9.grid(True, alpha=0.3)
    else:
        ax9.text(0.5, 0.5, "No predicted CTE data", transform=ax9.transAxes,
                 ha="center", va="center", fontsize=12, color="gray")
        ax9.set_title("Predicted vs Actual CTE (no data)")

# ── statistics ─────────────────────────────────────────────────────────
cte_rmse  = math.sqrt(np.mean(cte**2))
hdg_rmse  = math.sqrt(np.mean(hdg**2))
cte_max_v = float(np.max(np.abs(cte)))
n_recov   = int(recov_mask.sum())
n_reset   = int(kreset.sum())
pct_recov = n_recov / len(records) * 100

vel_err_rmse = 0.0
if has_new_fields:
    valid_ve = np.isfinite(vel_error)
    if valid_ve.sum() > 0:
        vel_err_rmse = math.sqrt(np.mean(vel_error[valid_ve]**2))

stats_text = (
    f"CTE RMSE={cte_rmse:.3f}m  |CTE|max={cte_max_v:.3f}m  "
    f"HDG RMSE={hdg_rmse:.2f}°  VelErr RMSE={vel_err_rmse:.2f}km/h  "
    f"RECOV={n_recov}ticks({pct_recov:.1f}%)  kappa_reset={n_reset}ticks  "
    f"total={len(records)}ticks/{t[-1]:.1f}s"
)
fig.suptitle(stats_text, fontsize=10, y=0.995)

# ── Summary stats box (JSON-level stats if available) ─────────────────
summary = log_data.get("summary", {})
if summary:
    box_text = (
        f"JSON Summary:\n"
        f"  CTE RMSE:     {summary.get('cte_rmse', 'N/A'):.4f} m\n"
        f"  HDG RMSE:     {summary.get('hdg_rmse_deg', 'N/A'):.2f} deg\n"
        f"  VelErr RMSE:  {summary.get('vel_err_rmse', 'N/A'):.2f} km/h\n"
        f"  Records:      {summary.get('num_records', 'N/A')}"
    ) if isinstance(summary.get('cte_rmse'), (int, float)) else str(summary)
    fig.text(0.02, 0.01, box_text, fontsize=8, family="monospace",
             bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.8),
             verticalalignment="bottom")

plt.savefig(out_path, dpi=150, bbox_inches="tight")
print(f"[analyze] saved : {out_path}")
print(f"[analyze] CTE RMSE={cte_rmse:.3f}m  HDG RMSE={hdg_rmse:.2f}°  max={cte_max_v:.3f}m  RECOV={pct_recov:.1f}%")
if has_new_fields:
    print(f"[analyze] VelErr RMSE={vel_err_rmse:.2f}km/h")
