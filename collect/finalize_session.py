#!/usr/bin/env python3
"""Turn ego_raw.jsonl into ego.csv (pack_session format) + route.json, and audit the session.

ego.csv columns: timestamp_us,x,y,z,qx,qy,qz,qw,heading_deg,speed_xy_mps,vx,accel,brake,wheel_angle
The heading->yaw convention is detected from motion direction and written to session.json.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

CONVENTIONS = {
    "yaw=heading": lambda h: h,
    "yaw=90-heading": lambda h: 90.0 - h,
    "yaw=-heading": lambda h: -h,
    "yaw=heading+180": lambda h: h + 180.0,
}


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def detect_convention(t_us, xy, heading_deg):
    dt = np.diff(t_us) / 1e6
    d = np.diff(xy, axis=0)
    speed = np.linalg.norm(d, axis=1) / np.maximum(dt, 1e-6)
    ok = (speed > 1.0) & (dt > 0)
    if ok.sum() < 20:
        return None, None, int(ok.sum())
    motion = np.degrees(np.arctan2(d[ok, 1], d[ok, 0]))
    h = heading_deg[:-1][ok]
    best, best_err = None, None
    errors = {}
    for name, fn in CONVENTIONS.items():
        err = float(np.median(np.abs(wrap_deg(fn(h) - motion))))
        errors[name] = err
        if best_err is None or err < best_err:
            best, best_err = name, err
    return best, errors, int(ok.sum())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("session", type=Path)
    p.add_argument("--force-convention", choices=list(CONVENTIONS))
    args = p.parse_args()
    session = args.session.resolve()
    raw = session / "ego_raw.jsonl"
    if not raw.is_file():
        sys.exit(f"missing {raw}")
    ego, routes, collisions, bad = [], [], [], 0
    for line in raw.read_text().splitlines():
        if not line.strip():
            continue
        try:
            r = json.loads(line)
        except ValueError:
            bad += 1
            continue
        kind = r.get("type")
        (ego if kind == "ego" else collisions if kind == "collision" else routes).append(r)
    if bad:
        print(f"경고: ego_raw.jsonl 손상 줄 {bad}개 건너뜀", file=sys.stderr)
    if len(ego) < 32:
        sys.exit(f"too few ego messages: {len(ego)}")
    ego.sort(key=lambda r: r["t_us"])
    # drop duplicate stamps (MORAI can repeat a stamp)
    dedup = [ego[0]]
    for r in ego[1:]:
        if r["t_us"] > dedup[-1]["t_us"]:
            dedup.append(r)
    ego = dedup
    t = np.array([r["t_us"] for r in ego], dtype=np.int64)
    xyz = np.array([[r["x"], r["y"], r["z"]] for r in ego])
    heading = np.array([r["heading_deg"] for r in ego])
    gaps_ms = np.diff(t) / 1e3
    conv, errors, n_moving = detect_convention(t, xyz[:, :2], heading)
    if args.force_convention:
        conv = args.force_convention
    audit = {
        "ego_raw_bad_lines": bad,
        "ego_messages": len(ego), "ego_hz": float(len(ego) / max((t[-1] - t[0]) / 1e6, 1e-6)),
        "ego_gap_ms_median": float(np.median(gaps_ms)), "ego_gap_ms_max": float(gaps_ms.max()),
        "ego_gaps_over_120ms": int((gaps_ms > 120).sum()),
        "moving_samples_for_heading_check": n_moving,
        "heading_convention_errors_deg": errors, "heading_convention": conv,
    }
    if conv is None:
        print(json.dumps(audit, indent=2))
        sys.exit("차량이 거의 안 움직여서 heading 방향 규약을 판정할 수 없음. 1m/s 이상으로 20샘플 이상 주행 필요 "
                 "(또는 --force-convention).")
    if errors and errors[conv] > 20:
        print(json.dumps(audit, indent=2))
        sys.exit(f"heading 규약 판정 실패: 최소 오차 {errors[conv]:.1f}deg > 20deg")
    yaw = np.radians(CONVENTIONS[conv](heading))
    speed_xy = np.concatenate([[0.0], np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1) / np.maximum(np.diff(t) / 1e6, 1e-6)])
    vx = np.array([r["vx"] for r in ego])
    # infer velocity.x unit by comparing with positional speed while moving
    mov = speed_xy > 1.0
    ratio = float(np.median(vx[mov] / speed_xy[mov])) if mov.sum() >= 20 else None
    audit["velocity_x_over_positional_speed_median"] = ratio
    audit["velocity_x_unit_guess"] = ("km/h" if ratio and abs(ratio - 3.6) < 0.5 else "m/s" if ratio and abs(ratio - 1) < 0.2 else "unknown")
    with (session / "ego.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp_us", "x", "y", "z", "qx", "qy", "qz", "qw", "heading_deg", "speed_xy_mps", "vx", "accel", "brake", "wheel_angle"])
        for i, r in enumerate(ego):
            half = yaw[i] / 2
            w.writerow([int(t[i]), f"{xyz[i,0]:.6f}", f"{xyz[i,1]:.6f}", f"{xyz[i,2]:.6f}", "0", "0",
                        f"{math.sin(half):.9f}", f"{math.cos(half):.9f}", f"{heading[i]:.4f}", f"{speed_xy[i]:.4f}",
                        f"{vx[i]:.4f}", r["accel"], r["brake"], r["wheel_angle"]])
    if routes:
        route = routes[-1]
        poses = np.array(route["poses_xyz_qxyzw"], dtype=np.float64)
        seg = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
        route_json = {"frame_id": route["frame_id"], "t_us": route["t_us"], "sha1": route["sha1"],
                      "poses_xyz_qxyzw": route["poses_xyz_qxyzw"], "point_count": len(poses),
                      "spacing_m_median": float(np.median(seg)) if len(seg) else None,
                      "length_m": float(seg.sum()), "distinct_route_messages": len(routes)}
        (session / "route.json").write_text(json.dumps(route_json))
        audit["route_points"] = len(poses)
        audit["route_length_m"] = route_json["length_m"]
        audit["route_spacing_m_median"] = route_json["spacing_m_median"]
        # distance from ego track to the route (sanity: same frame?)
        d = np.linalg.norm(xyz[::50, None, :2] - poses[None, :, :2], axis=2).min(axis=1)
        audit["ego_to_route_distance_m_median"] = float(np.median(d))
    else:
        audit["route_points"] = 0
    # camera audit against 10Hz grid (+-20ms), same rule as teacher/pack_session.py
    mf = session / "manifest.jsonl"
    cam = {}
    if mf.is_file():
        for line in mf.read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                cam.setdefault(r["view"], []).append(r["rgb_timestamp_ns"] // 1000)
        audit["camera"] = {}
        for v, ts in cam.items():
            ts = np.array(sorted(ts), dtype=np.int64)
            grid = np.arange(ts[0] + 100000, ts[-1], 100000)
            if len(grid):
                # Nearest frame for each grid tick, without a frames-by-ticks array.
                right = np.searchsorted(ts, grid)
                left_ts = ts[np.clip(right - 1, 0, len(ts) - 1)]
                right_ts = ts[np.clip(right, 0, len(ts) - 1)]
                err = np.minimum(np.abs(grid - left_ts), np.abs(right_ts - grid)) / 1e3
            else:
                err = np.array([0.0])
            dt = np.diff(ts) / 1e3
            audit["camera"][v] = {"frames": int(len(ts)), "hz": float(len(ts) / max((ts[-1] - ts[0]) / 1e6, 1e-6)),
                                  "dt_ms_median": float(np.median(dt)), "dt_ms_max": float(dt.max()),
                                  "grid10hz_within_20ms_pct": float((err <= 20).mean() * 100), "grid_err_ms_max": float(err.max())}
        cam_span = [(min(v), max(v)) for v in cam.values()]
        audit["camera_ego_overlap_s"] = float((min(e for _, e in cam_span) - max(s for s, _ in cam_span)) / 1e6)
        audit["ego_covers_camera"] = bool(t[0] <= max(s for s, _ in cam_span) and t[-1] >= min(e for _, e in cam_span))
    # MORAI streams CollisionData at ~50 Hz and lists the ego itself (unique_id 0, type -1, same pose) as an
    # "object"; only entries that are other objects count as a collision.
    def is_ego_entry(o, t_us):
        if o.get("type") != -1 and o.get("id") != 0:
            return False
        i = int(np.searchsorted(t, t_us))
        i = min(max(i, 0), len(t) - 1)
        return math.hypot(o.get("x", 1e9) - xyz[i, 0], o.get("y", 1e9) - xyz[i, 1]) < 1.5
    raw_collision_msgs = len(collisions)
    # MORAI also lists objects colliding with other things: pedestrians stuck in
    # scenery stay listed from t=0 for the whole run, 400 m away (auto runs 62, 144).
    # Ours only when an object's listing STARTS (episode onset, gap > 0.5 s) within
    # reach of our car; passing near a permanently listed pedestrian is not a contact.
    def ego_at(t_us):
        i = min(max(int(np.searchsorted(t, t_us)), 0), len(t) - 1)
        return xyz[i, 0], xyz[i, 1]
    last_seen, ours, filtered = {}, {}, []
    for r in sorted(collisions, key=lambda r: r["t_us"]):
        ex, ey = ego_at(r["t_us"])
        others = []
        for o in r["objects"]:
            if is_ego_entry(o, r["t_us"]):
                continue
            key = o.get("id")
            if key not in last_seen or r["t_us"] - last_seen[key] > 500_000:
                ours[key] = math.hypot(o.get("x", 0.) - ex, o.get("y", 0.) - ey) < 3.5
            last_seen[key] = r["t_us"]
            if ours[key]:
                others.append(o)
        if others:
            filtered.append({**r, "objects": others})
    collisions = filtered
    # cluster messages into events (gap > 1 s starts a new event); build_clips masks around them
    events = []
    for r in sorted(collisions, key=lambda r: r["t_us"]):
        if events and r["t_us"] - events[-1]["end_us"] <= 1_000_000:
            events[-1]["end_us"] = r["t_us"]
            events[-1]["messages"] += 1
            for o in r["objects"]:
                events[-1]["objects"].setdefault(str(o.get("id")), o.get("name"))
        else:
            events.append({"start_us": r["t_us"], "end_us": r["t_us"], "messages": 1,
                           "objects": {str(o.get("id")): o.get("name") for o in r["objects"]}})
    (session / "collisions.json").write_text(json.dumps({"events": events, "messages": len(collisions)}, ensure_ascii=False))
    audit["collision_messages_raw"] = raw_collision_msgs
    audit["collision_messages"] = len(collisions)
    audit["collision_events"] = len(events)
    audit["collision_seconds"] = round(sum((e["end_us"] - e["start_us"]) / 1e6 for e in events), 1)
    sj = session / "session.json"
    meta = json.loads(sj.read_text()) if sj.is_file() else {}
    meta["audit"] = audit
    sj.write_text(json.dumps(meta, indent=2))
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
