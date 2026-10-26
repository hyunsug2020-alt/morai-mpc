#!/usr/bin/env python3
"""Competition speed ladder: raise the speed 5 % per run, only after a clean run.

State lives in data/autodrive/competition_speed.json. Level k drives
  normal roads  : min(max_kmh, base_kmh * 1.05**k)   (was a fixed 25 km/h)
  curves        : follower curve_alat_max = min(alat_max, base_alat * 1.05**(2k))
                  (lateral accel grows with v**2, so curve speed also rises 5 %/level)
  highway zone  : max_kmh (55), unchanged.

  plan     --run-dir D   write D/speed_zones.json + D/speed_plan.json, print "normal_kmh alat level"
  evaluate --run-dir D   score D/mpc.csv, write D/speed_eval.json, advance the level on a pass
  set --level N | show
"""
import argparse
import csv
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATE = os.path.join(ROOT, "data/autodrive/competition_speed.json")
ASSETS = os.path.join(ROOT, "autodrive/student_v3_claude")   # E2E launch data (web_speed_zones.json)
DEFAULT = {
    "level": 0, "step_pct": 5.0, "base_kmh": 25.0, "max_kmh": 35.0,  # normal-road target; highway zone keeps 55
    "base_alat": 2.26, "alat_max": 3.5,
    # A run passes only if, while moving on the Student path (mode NORMAL, v > 3 km/h):
    "criteria": {"min_moving_s": 120.0, "cte_p95_m": 0.30, "cte_max_m": 0.60,
                 "steer_reversals_per_s": 0.30, "reach_ratio": 0.90},
    "history": [],
}


def load_state():
    state = dict(DEFAULT)
    try:
        with open(STATE) as source:
            state.update(json.load(source))
    except (OSError, ValueError):
        pass
    return state


def save_state(state):
    tmp = STATE + ".tmp"
    with open(tmp, "w") as target:
        json.dump(state, target, indent=2, ensure_ascii=False)
    os.replace(tmp, STATE)


def targets(state, level=None):
    level = state["level"] if level is None else level
    factor = (1.0 + state["step_pct"] / 100.0) ** level
    normal = min(state["max_kmh"], state["base_kmh"] * factor)
    alat = min(state["alat_max"], state["base_alat"] * factor * factor)
    return round(normal, 2), round(alat, 3)


def plan(args):
    state = load_state()
    normal, alat = targets(state)
    with open(os.path.join(ASSETS, "web_speed_zones.json")) as source:
        zones = json.load(source)          # highway zone(s) keep their own speed
    # First match wins in SpeedZones, so the whole-route entry goes last.
    # A zone keeps its own speed, also below the ladder: the guardrail S-curve cap was
    # raised from 32 to the ladder 46.5 km/h and the car touched the guardrail (run 210).
    zones = [dict(z, kmh=float(z["kmh"])) for z in zones]
    zones.append({"from_pct": 0, "to_pct": 100, "kmh": normal, "note": "competition speed ladder"})
    with open(os.path.join(args.run_dir, "speed_zones.json"), "w") as target:
        json.dump(zones, target, indent=2)
    with open(os.path.join(args.run_dir, "speed_plan.json"), "w") as target:
        json.dump({"level": state["level"], "normal_kmh": normal, "curve_alat_max": alat,
                   "zones": zones}, target, indent=2)
    print(f"{normal} {alat} {state['level']}")


def reversal_rate(times, steer):
    """Steering direction reversals per second (ignoring < 0.05 deg steps): oscillation index."""
    flips, last_sign = 0, 0
    for i in range(1, len(steer)):
        delta = steer[i] - steer[i - 1]
        if abs(delta) < math.radians(0.05):
            continue
        sign = 1 if delta > 0 else -1
        if last_sign and sign != last_sign:
            flips += 1
        last_sign = sign
    duration = times[-1] - times[0] if len(times) > 1 else 0.0
    return flips / duration if duration > 0 else 0.0


def percentile(values, q):
    values = sorted(values)
    if not values:
        return float("nan")
    return values[min(len(values) - 1, int(round(q / 100.0 * (len(values) - 1))))]


SECTION_NAMES = {"static": "정적", "dynamic": "동적", "tunnel": "터널", "overpass": "고가도로",
                 "signal": "신호등", "intersection": "교차로", "normal": "일반"}


def section_metrics(rows):
    """CTE per mission section from data/autodrive/drivable_rules.json (route distance in m)."""
    try:
        with open(os.path.join(ROOT, "data/autodrive/drivable_rules.json")) as source:
            sections = json.load(source).get("sections") or []
        with open(os.path.join(ASSETS, "global_path.json")) as source:
            route = [(float(p["x"]), float(p["y"])) for p in json.load(source)["waypoints"]]
    except (OSError, ValueError, KeyError):
        return {}
    if not sections or len(route) < 2:
        return {}
    station = [0.0]
    for (ax, ay), (bx, by) in zip(route, route[1:]):
        station.append(station[-1] + math.hypot(bx - ax, by - ay))
    grid = {}
    for i, (x, y) in enumerate(route):
        grid.setdefault((int(x // 10), int(y // 10)), []).append(i)

    def station_of(x, y):
        best, best_d = None, 400.0
        for gx in range(int(x // 10) - 2, int(x // 10) + 3):
            for gy in range(int(y // 10) - 2, int(y // 10) + 3):
                for i in grid.get((gx, gy), ()):
                    d = (route[i][0] - x) ** 2 + (route[i][1] - y) ** 2
                    if d < best_d:
                        best, best_d = i, d
        return None if best is None else station[best]

    out = {}
    stations = [station_of(r[5], r[6]) for r in rows]
    for section in sections:
        sel = [abs(r[2]) for r, s in zip(rows, stations)
               if s is not None and section["start_m"] <= s <= section["end_m"]]
        if len(sel) < 20:
            continue
        name = f"{SECTION_NAMES.get(section['type'], section['type'])} {section['start_m']:.0f}-{section['end_m']:.0f}m"
        if section.get("label"):
            name += f" ({section['label']})"
        out[name] = {"n": len(sel), "cte_p95_m": round(percentile(sel, 95), 3), "cte_max_m": round(max(sel), 3)}
    return out


def evaluate(args):
    state = load_state()
    crit = state["criteria"]
    level = state["level"]
    run_plan = {}
    try:
        with open(os.path.join(args.run_dir, "speed_plan.json")) as source:
            run_plan = json.load(source)
            level = run_plan["level"]
    except (OSError, ValueError, KeyError):
        pass
    normal, alat = targets(state, level)
    # A requested speed change applies to the next launch. Score the running
    # session against its own saved targets, and never promote the new setting
    # based on a run that used the previous speed.
    normal = float(run_plan.get("normal_kmh", normal))
    alat = float(run_plan.get("curve_alat_max", alat))
    rows = []
    with open(os.path.join(args.run_dir, "mpc.csv")) as source:
        for row in csv.DictReader(source):
            try:
                if row["mode"] != "NORMAL":
                    continue
                t, v, cte = float(row["t"]), float(row["v_kmh"]), float(row["cte"])
                steer, kappa = float(row["steer_cmd"]), float(row["path_curvature"] or 0)
                x, y = float(row["x"]), float(row["y"])
            except (KeyError, ValueError):
                continue
            if v > 3.0 and math.isfinite(cte):
                rows.append((t, v, cte, steer, kappa, x, y))
    result = {"run_dir": args.run_dir, "level": level, "normal_kmh": normal, "curve_alat_max": alat,
              "evaluated_at": time.time()}
    if len(rows) < 50:
        result.update(verdict="hold", reason="too little driving to judge")
    else:
        dt = [min(0.2, b[0] - a[0]) for a, b in zip(rows, rows[1:])]
        moving_s = sum(d for d in dt if d > 0)
        abs_cte = [abs(r[2]) for r in rows]
        straight = [r[2] for r in rows if abs(r[4]) < 0.003]
        curve = [abs(r[2]) for r in rows if abs(r[4]) >= 0.01]
        fast = [r for r in rows if r[1] >= 0.8 * normal]
        metrics = {
            "moving_s": round(moving_s, 1),
            "v_max_kmh": round(max(r[1] for r in rows), 1),
            "fast_share": round(len(fast) / len(rows), 3),
            "cte_p95_m": round(percentile(abs_cte, 95), 3),
            "cte_max_m": round(max(abs_cte), 3),
            "cte_curve_p95_m": round(percentile(curve, 95), 3) if curve else None,
            "cte_straight_bias_m": round(sum(straight) / len(straight), 3) if straight else None,
            "steer_reversals_per_s": round(reversal_rate([r[0] for r in rows], [r[3] for r in rows]), 3),
        }
        by_speed = {}
        for lo in range(0, 60, 10):
            sel = [abs(r[2]) for r in rows if lo <= r[1] < lo + 10]
            if len(sel) >= 30:
                by_speed[f"{lo}-{lo + 10}"] = {"n": len(sel), "cte_p95_m": round(percentile(sel, 95), 3),
                                               "cte_max_m": round(max(sel), 3)}
        metrics["by_speed"] = by_speed
        metrics["by_section"] = section_metrics(rows)
        failures = []
        if metrics["cte_p95_m"] > crit["cte_p95_m"]:
            failures.append(f"CTE p95 {metrics['cte_p95_m']} > {crit['cte_p95_m']} m")
        if metrics["cte_max_m"] > crit["cte_max_m"]:
            failures.append(f"CTE max {metrics['cte_max_m']} > {crit['cte_max_m']} m")
        if metrics["steer_reversals_per_s"] > crit["steer_reversals_per_s"]:
            failures.append(f"steering oscillation {metrics['steer_reversals_per_s']}/s > {crit['steer_reversals_per_s']}/s")
        reached = metrics["v_max_kmh"] >= crit["reach_ratio"] * normal
        if failures:
            result.update(verdict="fail", reason="; ".join(failures))
        elif moving_s < crit["min_moving_s"]:
            result.update(verdict="hold", reason=f"only {moving_s:.0f} s moving (< {crit['min_moving_s']:.0f} s)")
        elif not reached:
            result.update(verdict="hold", reason=f"never reached {crit['reach_ratio'] * normal:.1f} km/h")
        else:
            result.update(verdict="pass", reason="clean run")
        result["metrics"] = metrics
    if (result["verdict"] == "pass" and level == state["level"]
            and (normal, alat) == targets(state) and normal < state["max_kmh"]):
        state["level"] = level + 1
    result["next_level"] = state["level"]
    result["next_normal_kmh"], result["next_curve_alat_max"] = targets(state)
    state["history"] = (state.get("history") or [])[-199:] + [
        {k: result.get(k) for k in ("run_dir", "level", "normal_kmh", "verdict", "reason", "evaluated_at")}]
    save_state(state)
    with open(os.path.join(args.run_dir, "speed_eval.json"), "w") as target:
        json.dump(result, target, indent=2, ensure_ascii=False)
    print(f"speed ladder level {level} ({normal} km/h, alat {alat}): {result['verdict'].upper()} - {result['reason']}"
          f" -> next level {result['next_level']} ({result['next_normal_kmh']} km/h)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    for name in ("plan", "evaluate"):
        p = sub.add_parser(name)
        p.add_argument("--run-dir", required=True)
    p = sub.add_parser("set")
    p.add_argument("--level", type=int, required=True)
    sub.add_parser("show")
    args = parser.parse_args()
    if args.cmd == "plan":
        plan(args)
    elif args.cmd == "evaluate":
        evaluate(args)
    elif args.cmd == "set":
        state = load_state()
        state["level"] = max(0, args.level)
        save_state(state)
        print(json.dumps(dict(level=state["level"], zip=targets(state))))
    else:
        state = load_state()
        normal, alat = targets(state)
        print(json.dumps({"level": state["level"], "normal_kmh": normal, "curve_alat_max": alat,
                          "last": (state.get("history") or [None])[-1]}, ensure_ascii=False))


if __name__ == "__main__":
    sys.exit(main())
