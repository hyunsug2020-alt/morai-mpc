#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_run.py — mpc_log.json + mixed.json → 진단 plot + 텍스트 요약
Usage: analyze_run.py <log.json> <path.json> <out.png>
"""
import json
import sys
import statistics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def load_log(p):
    with open(p) as f:
        d = json.load(f)
    return d.get('records', []), d.get('summary', {})


def load_path(p):
    with open(p) as f:
        d = json.load(f)
    wps = d['waypoints'] if isinstance(d, dict) else d
    return [(w['x'], w['y']) for w in wps]


def main(log_path, path_path, out_png):
    recs, summary = load_log(log_path)
    if not recs:
        print("EMPTY LOG"); sys.exit(1)
    wps = load_path(path_path)

    t = np.array([r['t'] for r in recs])
    x = np.array([r['x'] for r in recs])
    y = np.array([r['y'] for r in recs])
    cte = np.array([r['cte'] for r in recs])
    hdg = np.array([r['hdg_err_deg'] for r in recs])
    v = np.array([r.get('v_kmh', 0) for r in recs])
    tv = np.array([r.get('target_vel', 0) for r in recs])
    steer = np.array([r.get('steer_cmd', 0) for r in recs])
    solver_us = np.array([r.get('rti_solver_us', 0) for r in recs])
    wx = np.array([w[0] for w in wps])
    wy = np.array([w[1] for w in wps])

    fig, axs = plt.subplots(3, 2, figsize=(14, 10))

    # 1. 경로 vs 실제 trajectory
    ax = axs[0, 0]
    ax.plot(wx, wy, 'b-', linewidth=0.5, label='ref path', alpha=0.5)
    ax.plot(x, y, 'r-', linewidth=1.2, label='actual')
    ax.scatter(x[0], y[0], c='g', s=60, marker='o', label='start', zorder=5)
    ax.scatter(x[-1], y[-1], c='k', s=60, marker='x', label='end', zorder=5)
    ax.set_aspect('equal')
    ax.set_title(f"Trajectory ({len(recs)} pts)")
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.legend(); ax.grid(alpha=0.3)

    # 2. CTE 시계열
    ax = axs[0, 1]
    ax.plot(t, cte, 'r-', linewidth=0.8)
    ax.axhline(0, color='k', linewidth=0.5)
    ax.fill_between(t, -0.5, 0.5, alpha=0.1, color='g')
    ax.set_title(f"CTE (rmse={summary.get('cte_rmse',0):.3f}m, |max|={max(abs(cte)):.2f}m)")
    ax.set_xlabel('t [s]'); ax.set_ylabel('cte [m]'); ax.grid(alpha=0.3)

    # 3. Heading error
    ax = axs[1, 0]
    ax.plot(t, hdg, 'b-', linewidth=0.8)
    ax.axhline(0, color='k', linewidth=0.5)
    ax.set_title(f"Heading err (rmse={summary.get('hdg_rmse_deg',0):.2f}°, |max|={max(abs(hdg)):.1f}°)")
    ax.set_xlabel('t [s]'); ax.set_ylabel('hdg [deg]'); ax.grid(alpha=0.3)

    # 4. 속도 (actual vs target)
    ax = axs[1, 1]
    ax.plot(t, v, 'r-', linewidth=0.8, label='actual')
    ax.plot(t, tv, 'g--', linewidth=0.8, label='target')
    ax.set_title(f"Velocity (vel_err_rmse={summary.get('vel_err_rmse',0):.2f})")
    ax.set_xlabel('t [s]'); ax.set_ylabel('v [km/h]')
    ax.legend(); ax.grid(alpha=0.3)

    # 5. Steering cmd
    ax = axs[2, 0]
    ax.plot(t, steer, 'm-', linewidth=0.8)
    ax.set_title(f"Steer cmd (|max|={max(abs(steer)):.3f})")
    ax.set_xlabel('t [s]'); ax.set_ylabel('steer'); ax.grid(alpha=0.3)

    # 6. Solver time
    ax = axs[2, 1]
    if solver_us.max() > 0:
        ax.plot(t, solver_us / 1000.0, 'c-', linewidth=0.8)
        ax.axhline(200, color='r', linewidth=0.5, label='200ms budget')
        ax.set_title(f"RTI solver time (mean={solver_us.mean()/1000:.1f}ms, max={solver_us.max()/1000:.1f}ms)")
        ax.set_xlabel('t [s]'); ax.set_ylabel('time [ms]')
        ax.legend()
    else:
        ax.text(0.5, 0.5, 'no solver timing', ha='center', va='center', transform=ax.transAxes)
    ax.grid(alpha=0.3)

    fig.suptitle(f"{log_path.split('/')[-1]}  |  cte_rmse={summary.get('cte_rmse',0):.3f}m  hdg_rmse={summary.get('hdg_rmse_deg',0):.2f}°", fontsize=11)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    print(f"saved: {out_png}")

    # 텍스트 진단
    print("=" * 50)
    print(f"  cte_rmse  : {summary.get('cte_rmse',0):.3f} m   (target <0.5)")
    print(f"  hdg_rmse  : {summary.get('hdg_rmse_deg',0):.2f} deg (target <5)")
    print(f"  vel_err   : {summary.get('vel_err_rmse',0):.2f}")
    print(f"  records   : {len(recs)}")
    print(f"  duration  : {t[-1]-t[0]:.1f} s")
    print(f"  vel mean  : {v.mean():.1f} km/h (target mean {tv.mean():.1f})")
    if solver_us.max() > 0:
        print(f"  solver max: {solver_us.max()/1000:.1f} ms (budget 200)")
    # 진단
    issues = []
    if summary.get('cte_rmse', 0) > 1.0:
        issues.append(f"CTE 큼 → weight_position 증가 / lookahead 조정")
    if summary.get('hdg_rmse_deg', 0) > 10:
        issues.append(f"HDG 큼 → weight_heading 증가")
    if v.mean() < 1.0:
        issues.append(f"차량 정지 → 자율주행 모드 미진입 (ctrl_mode 확인)")
    if solver_us.max() / 1000 > 180:
        issues.append(f"solver 시간 초과 → horizon 감소 또는 Release 빌드 확인")
    print("DIAGNOSE:")
    for i in issues:
        print(f"  ⚠ {i}")
    if not issues:
        print("  ✓ no major issues")


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3])
