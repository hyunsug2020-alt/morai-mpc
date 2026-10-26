#!/usr/bin/env python3
"""Normal-road speed guard for auto collection (user 2026-10-05: up to 55 km/h outside the
highway, but never a collision).

After every finished run: any contact -> two ladder levels down (floor LEVEL_MIN);
a completed lap without contact -> one level up (ceiling LEVEL_MAX, 55 km/h).
Contact = a MORAI collision whose listing starts within 3.5 m (pedestrian/obstacle) or
6 m (vehicle) of us, or a LiDAR body gap <= 0.1 m in the decision log (map scenery such
as drums and walls is never listed by MORAI).

Run in the background: nohup setsid python3 speed_guard.py &   (log data/autodrive/speed_guard.log)
"""
import glob
import json
import math
import os
import time

ROOT = "/home/david/morai_competition"
STATE = f"{ROOT}/data/autodrive/competition_speed.json"
RUNS = f"{ROOT}/data/autodrive/auto_collect/runs.jsonl"
LOG = f"{ROOT}/data/autodrive/speed_guard.log"
LEVEL_MIN, LEVEL_MAX = 9, 13          # 46.5 .. 55 km/h; contacts now include the LiDAR body gap (guardrail etc.)


def log(**kw):
    with open(LOG, "a") as f:
        f.write(json.dumps(dict(t=time.strftime("%F %T"), **kw), ensure_ascii=False) + "\n")


def contacts(sid):
    found = []
    try:
        ego, col = [], []
        for line in open(f"{ROOT}/data/raw/{sid}/ego_raw.jsonl"):
            if '"collision"' in line:
                col.append(json.loads(line))
            elif '"ego"' in line:
                r = json.loads(line)
                ego.append((r["t_us"], r["x"], r["y"]))
        import bisect
        ts = [e[0] for e in ego]
        last = {}
        for r in sorted(col, key=lambda r: r["t_us"]):
            e = ego[min(bisect.bisect(ts, r["t_us"]), len(ego) - 1)]
            for o in r["objects"]:
                if o.get("type") == -1 or o.get("id") == 0:
                    continue
                k = o.get("id")
                if k not in last or r["t_us"] - last[k] > 500000:
                    reach = 6. if o.get("type") == 1 else 3.5
                    if math.hypot(o["x"] - e[1], o["y"] - e[2]) < reach:
                        found.append(f"morai:{o.get('name')}")
                last[k] = r["t_us"]
    except OSError:
        pass
    for f in glob.glob(f"{ROOT}/data/autodrive/mpc_runs/{sid}/*/student_gui_{sid}.jsonl"):
        for line in open(f):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            g = r.get("body_gap_m")
            if g is not None and g <= .1 and r.get("speed_kmh", 0) > 1.:
                found.append(f"lidar:{g}@({r['ego'][0]:.0f},{r['ego'][1]:.0f})")
                break
    return found


def main():
    seen = set()
    try:
        for line in open(RUNS):
            seen.add(json.loads(line).get("sid"))
    except OSError:
        pass
    log(msg="started", known_runs=len(seen))
    while True:
        try:
            for line in open(RUNS):
                r = json.loads(line)
                sid = r.get("sid")
                if sid in seen:
                    continue
                seen.add(sid)
                time.sleep(20)                     # let the recorders flush
                hit = contacts(sid)
                state = json.load(open(STATE))
                old = int(state["level"])
                if hit:
                    new = max(LEVEL_MIN, old - 2)
                elif r.get("outcome") == "lap_complete":
                    new = min(LEVEL_MAX, old + 1)
                else:
                    new = old
                if new != old:
                    state["level"] = new
                    tmp = STATE + ".tmp"
                    json.dump(state, open(tmp, "w"), indent=2, ensure_ascii=False)
                    os.replace(tmp, STATE)
                log(sid=sid, outcome=r.get("outcome"), contacts=hit[:5], level=f"{old}->{new}",
                    kmh=round(min(state["max_kmh"], state["base_kmh"] * 1.05 ** new), 1))
        except Exception as error:      # never die on a malformed line
            log(msg="error", error=str(error))
        time.sleep(15)


if __name__ == "__main__":
    main()
