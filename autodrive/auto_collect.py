#!/usr/bin/env python3
"""Unattended v6 data collection on the single-PC stack.

Loop until stopped: generate a competition-mode scenario in Scenario Studio,
load it into MORAI over gRPC, drive one lap (max 6 min) with the dashboard's
sensor collection (the E2E launch student_v3_follow_claude.sh), then move on.

Stop: `systemctl --user stop morai-auto-collect` or touch data/autodrive/auto_collect/stop
(the current lap is stopped cleanly either way).
"""
import json
import math
import os
import random
import shutil
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
RAW = DATA / "raw"
STATE = DATA / "autodrive/auto_collect"
STOP_FILE = STATE / "stop"
RESTART_FILE = STATE / "restart"   # touch to reload this script after the current lap
WEB = os.environ.get("AUTO_COLLECT_WEB", "http://127.0.0.1:8767")
STUDIO = os.environ.get("AUTO_COLLECT_STUDIO", "http://127.0.0.1:4173")
SCENARIO_DIR = Path(os.environ.get("AUTO_COLLECT_SCENARIO_DIR", str(
    Path.home() / "MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025")))
PLAYER_LOG = Path(os.environ.get("MORAI_PLAYER_LOG", str(Path.home() / ".config/unity3d/MORAI/Simulator/Player.log")))
RPC_CONTAINER = "alpha_semantic_b0_web"
SINGLE_PC_NETWORK = ROOT / "morai/final_sensor/2026_molit_comp_network_single_pc.json"
RPC_CODE = ROOT / "semantic/runtime"
ASSETS = ROOT / "autodrive/student_v3_claude"     # route / HD-map / tunnel data of the E2E launch
JUDGE_DIR = ROOT / "autodrive/judge"               # optional self-judge (removed 2026-10-11); skipped when absent
LAP_MINUTES = float(os.environ.get("AUTO_COLLECT_MINUTES", "12"))   # 12 min per lap (user 2026-10-05): dynamic-zone mix made competition laps 10-11 min
MIN_FREE_GB = float(os.environ.get("AUTO_COLLECT_MIN_FREE_GB", "5"))
STUCK_S = 150.   # NPC queues at the first junction (idx ~390) stood >60 s; waiting behind them is right (runs 244, 254)
TEMPLATES = [f"morai_hard_{n:02d}" for n in range(8, 14)]
STATIC_KINDS = ["scatter", "slalom", "workzone", "parked", "bottleneck"]

stopping = False


def log(message, **extra):
    record = dict(t=time.strftime("%Y-%m-%d %H:%M:%S"), msg=message, **extra)
    print(json.dumps(record, ensure_ascii=False), flush=True)


def write_status(**values):
    STATE.mkdir(parents=True, exist_ok=True)
    path = STATE / "status.json"
    current = {}
    try:
        current = json.loads(path.read_text())
    except (OSError, ValueError):
        pass
    current.update(values, updated_at=time.time())
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(current, ensure_ascii=False, indent=2))
    os.replace(tmp, path)


def http(url, body=None, timeout=60):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(url, data=data, method="GET" if body is None else "POST",
                                     headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode())


def should_stop():
    return stopping or STOP_FILE.exists()


def route_length():
    raw = json.loads((ASSETS / "global_path.json").read_text())
    points = raw.get("waypoints", raw.get("path", raw)) if isinstance(raw, dict) else raw
    return len(points)


# ---------------------------------------------------------------- scenario
def random_config(rng, index):
    """Many situations, all inside the planner's obstacle sections (static obstacles are
    ignored elsewhere, as in the competition): route idx 480-1100 = studio s 245-545 m
    (boxes + pedestrians), idx 3760-4030 = s 1875-2000 m (tunnel)."""
    common = {
        "seed": f"auto-{time.strftime('%Y%m%d-%H%M%S')}-{index}", "template": rng.choice(TEMPLATES),
        "preset": "mixed", "corridor": 3.5, "gap": rng.choice([6, 8, 10, 12, 15, 20]), "egoClearance": 25,
        "passage": rng.choice([2.4, 2.6, 2.8, 3.2, 3.6]), "pedestrianSpeed": rng.choice([2, 2.5, 3, 3.5, 4]),   # around the studio default walking pace (user 2026-10-05)
        "activeDistance": rng.choice([18, 22, 26]),   # 25-45 crossed too early (user); 12-20 put our stopped bumper on their line (runs 60/68/86)
        "placementVersion": 2, "lanePolicyVersion": 2, "missionDistribution": rng.choice(["lane", "spread"]),
        "keepPedestrians": False, "keepObjects": False,
    }
    # Tunnel obstacles only come from competition mode: free placement requires a
    # 2-lane road and the HD map has the tunnel as one lane ("no placeable space").
    mode = rng.choice(["competition", "box"])
    # A theme per scenario (user 2026-10-05: varied obstacles and situations, competition
    # style): 1-3 dominant static situation kinds instead of the same even mix every lap.
    focus = rng.sample(STATIC_KINDS, rng.randint(1, 3))
    if mode == "competition":
        kinds = focus + ["crossing"]
        return dict(common, mode=mode, theme=focus, events=10, start=204, end=1990, keepTraffic=False, kinds=kinds,
                    missionPlan="competition-signals-tunnel-v2",
                    missionCounts={"static": rng.randint(3, 10), "dynamic": rng.randint(2, 7)})   # random counts every lap (user 2026-10-05)
    kinds = focus + (["crossing"] if rng.random() < .6 else []) + (["shoulder"] if rng.random() < .8 else [])
    # Random window inside the planner's obstacle section (s 245-545) and random count
    # (user 2026-10-05: many places, random amounts, but not so many the PC chokes).
    start = round(rng.uniform(245., 330.))
    end = min(545, start + round(rng.uniform(150., 300.)))
    return dict(common, mode=mode, theme=focus, events=rng.randint(8, 28), start=start, end=end,
                keepTraffic=rng.random() < .5, kinds=kinds, missionPlan=None, missionCounts=None)


def make_scenario(rng, index):
    for attempt in range(12):
        config = random_config(rng, index * 100 + attempt)
        try:
            studio_config = {k: v for k, v in config.items() if k not in ("mode", "theme")}
            result = http(STUDIO + "/api/save-to-morai", {"config": studio_config, "count": 1,
                                                          "cycleTemplates": False, "diversify": False}, timeout=120)
            break
        except urllib.error.HTTPError as error:
            if error.code != 400:      # 400 = this random layout does not fit; draw another
                raise
    else:
        raise RuntimeError("시나리오 생성 12회 실패")
    name = result["saved"][0]
    name = name if name.endswith(".json") else name + ".json"
    # The studio pairs each scenario with a ROS-bridge network (_MN.json) that MORAI applies on load,
    # after which SetVehicleNetwork back to UDP fails. Pair it with the single-PC UDP network instead.
    shutil.copyfile(SINGLE_PC_NETWORK, SCENARIO_DIR / "EgoNetwork" / f"{Path(name).stem}_MN.json")
    if config.get("mode") == "box":
        # Box-section layouts carry no tunnel obstacles (free placement needs 2 lanes):
        # borrow the tunnel objects of a competition-mode layout (user 2026-10-05).
        try:
            config["tunnel_objects"] = add_tunnel_obstacles(SCENARIO_DIR / name, config, rng)
        except Exception as error:
            log("tunnel obstacle merge failed", error=str(error)[-200:])
    if config.get("mode") == "competition" and rng.random() < .75:
        try:
            config["dynamic_mix"] = add_dynamic_zone_mix(SCENARIO_DIR / name, config, rng)
        except Exception as error:
            log("dynamic zone mix failed", error=str(error)[-200:])
    if rng.random() < .7:
        try:
            config["highway_npcs"] = add_highway_npcs(SCENARIO_DIR / name, rng)
        except Exception as error:
            log("highway NPC injection failed", error=str(error)[-200:])
    try:
        config["traffic_ends_moved"] = move_traffic_ends(SCENARIO_DIR / name)
    except Exception as error:
        log("traffic end move failed", error=str(error)[-200:])
    try:
        config["tunnel_slalom"] = slalom_tunnel_obstacles(SCENARIO_DIR / name, rng)
    except Exception as error:
        log("tunnel slalom failed", error=str(error)[-200:])
    try:
        config["lane_walkers_removed"] = drop_lane_walkers(SCENARIO_DIR / name)
    except Exception as error:
        log("lane-walker filter failed", error=str(error)[-200:])
    try:
        # Big vehicles stay (user 2026-10-05); every cluster keeps a way through instead.
        config["passable_shifts"] = keep_road_passable(SCENARIO_DIR / name, studio_model_info())
    except Exception as error:
        log("passability fix failed", error=str(error)[-200:])
    try:
        config["trimmed"] = cap_actors(SCENARIO_DIR / name, rng)
    except Exception as error:
        log("actor cap failed", error=str(error)[-200:])
    return name, config



# Route-lateral usable road for the guaranteed gap: inside the centre line .. the lane's
# right edge. Counting the shoulder (-3.0) opened gaps there and sent the car along the
# fence (user 2026-10-05: "why does it keep hitting the fence").
PASS_LEFT_M, PASS_RIGHT_M = 1.6, -1.75   # inside our lane: no shoulder or centre-line passing (user 2026-10-05, rule 3-2)
PASS_NEED_M = 2.7                        # free width to leave: body 1.9 m + 0.4 m each side
MODEL_HALF_WIDTH = {"vehicle": 1.0, "bike": .4, "barrier": .35, "barrel": None, "box": None}


def drop_lane_walkers(scenario_path):
    """Remove pedestrians that walk ALONG our lane (path parallel to the route, within
    2.5 m of it). Competition pedestrians cross the road; a "crossing" walker placed along
    the lane caught up with us from behind while we waited for a gap and walked into the
    car (auto run 216). Shoulder walkers (> 2.5 m) and real crossings stay."""
    import numpy as np
    route = highway_lanes()[0]
    tang = np.gradient(route, axis=0); tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    scene = json.loads(Path(scenario_path).read_text())
    paths = scene.get("waypointDataList", [])
    keep, removed = [], []
    for o in scene.get("pedestrianList", []):
        p = o.get("pos") or {}
        try:
            start = (float(p["x"]), float(p["y"]))
        except (KeyError, TypeError, ValueError):
            keep.append(o)
            continue
        pts = np.array([start] + path_points(paths, o["UNIQUEID"]))
        if len(pts) < 2 or np.linalg.norm(pts[-1] - pts[0]) < 3.:
            keep.append(o)
            continue
        j = np.argmin(np.linalg.norm(route[None, :, :] - pts[:, None, :], axis=2), axis=1)
        lateral = np.abs(np.einsum("ij,ij->i", pts - route[j], np.column_stack((-tang[j, 1], tang[j, 0]))))
        heading = (pts[-1] - pts[0]) / np.linalg.norm(pts[-1] - pts[0])
        along = abs(float(heading @ tang[j[0]]))
        if lateral.max() < 2.5 and along > .8:
            removed.append(dict(id=o["UNIQUEID"], x=round(start[0], 1), y=round(start[1], 1)))
            continue
        keep.append(o)
    if removed:
        scene["pedestrianList"] = keep
        Path(scenario_path).write_text(json.dumps(scene, ensure_ascii=False, indent=2))
    return removed


def keep_road_passable(scenario_path, model_info):
    """Every obstacle cluster must leave PASS_NEED_M of road: a 9 m truck parked across
    the lane left no way past without the oncoming lane (auto run 177). Big vehicles
    stay (user 2026-10-05: keep them, but there must be a way through); the cluster is
    pushed toward whichever road side needs the smaller shift. Planner obstacle section
    only (route idx 480-1100); the 12 m tunnel has room."""
    import numpy as np
    route = highway_lanes()[0]
    seg = route[480:1101]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))]
    tang = np.gradient(seg, axis=0); tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    norm = np.column_stack((-tang[:, 1], tang[:, 0]))
    scene = json.loads(Path(scenario_path).read_text())
    items = []
    for o in scene.get("objectList", []):
        q = np.array([float(o["pos"]["x"]), float(o["pos"]["y"])])
        j = int(np.argmin(np.linalg.norm(seg - q, axis=1)))
        if j in (0, len(seg) - 1):
            continue
        lat = float((q - seg[j]) @ norm[j])
        if abs(lat) > 6.:
            continue
        label, radius, cat = model_info.get(o.get("DataID"), ("?", 1., "box"))
        if cat == "vehicle" and radius >= 4.:
            half = 1.25                                  # heavy truck / bus
        else:
            half = MODEL_HALF_WIDTH.get(cat) or float(radius)
        items.append(dict(o=o, s=float(arc[j]), lat=lat, half=half, j=j))
    items.sort(key=lambda it: it["s"])
    moved, k = [], 0
    while k < len(items):
        group = [items[k]]
        while k + len(group) < len(items) and items[k + len(group)]["s"] - group[-1]["s"] < 6.:
            group.append(items[k + len(group)])
        k += len(group)
        def widest_gap(g):
            spans = sorted((max(PASS_RIGHT_M, it["lat"] - it["half"]), min(PASS_LEFT_M, it["lat"] + it["half"]))
                           for it in g if it["lat"] - it["half"] < PASS_LEFT_M and it["lat"] + it["half"] > PASS_RIGHT_M)
            edge, best = PASS_RIGHT_M, 0.
            for lo, hi in spans:
                best = max(best, lo - edge); edge = max(edge, hi)
            return max(best, PASS_LEFT_M - edge)
        if widest_gap(group) >= PASS_NEED_M:
            continue
        # Open the left side (push right) or the right side (push left): smaller total shift.
        push_right = [min(0., (PASS_LEFT_M - PASS_NEED_M) - (it["lat"] + it["half"])) for it in group]
        push_left = [max(0., (PASS_RIGHT_M + PASS_NEED_M) - (it["lat"] - it["half"])) for it in group]
        # Prefer opening the lane by pushing obstacles toward the shoulder (car stays in
        # its lane, away from the fence); the other way only when far cheaper.
        shifts = push_right if sum(map(abs, push_right)) <= 1.5 * sum(map(abs, push_left)) else push_left
        for it, d in zip(group, shifts):
            if abs(d) < 1e-3:
                continue
            q = np.array([float(it["o"]["pos"]["x"]), float(it["o"]["pos"]["y"])]) + d * norm[it["j"]]
            it["o"]["pos"].update(x=float(q[0]), y=float(q[1]), _x=f"{q[0]:.3f}", _y=f"{q[1]:.3f}")
            moved.append(dict(id=it["o"]["UNIQUEID"], shift=round(d, 2), s=round(it["s"], 1)))
    if moved:
        Path(scenario_path).write_text(json.dumps(scene, ensure_ascii=False, indent=2))
    return moved


def studio_model_info():
    """DataID -> (label, radius, category) from the studio's MODEL_INFO table."""
    import re
    text = (Path.home() / "morai-scenario-studio/public/engine.js").read_text()
    return {int(m.group(1)): (m.group(2), float(m.group(3)), m.group(4))
            for m in re.finditer(r"(\d{8}): \['([^']*)', ([0-9.]+), '([a-z]+)'\]", text)}

MAX_OBJECTS, MAX_PEDESTRIANS, MAX_VEHICLES = 35, 14, 22   # keep fusion/planning load bounded (user 2026-10-05)


def cap_actors(scenario_path, rng):
    """Drop randomly chosen added actors (mix 40000+, then tunnel 30000+, then highway
    NPCs 20000+) above the per-kind limits; studio mission actors stay."""
    scene = json.loads(Path(scenario_path).read_text())
    removed = []
    for key, limit in (("objectList", MAX_OBJECTS), ("pedestrianList", MAX_PEDESTRIANS), ("vehicleList", MAX_VEHICLES)):
        items = scene.get(key, [])
        while len(items) > limit:
            pool = ([o for o in items if o["UNIQUEID"] >= 40000] or [o for o in items if o["UNIQUEID"] >= 30000]
                    or [o for o in items if o["UNIQUEID"] >= 20000])
            if not pool:
                break
            victim = rng.choice(pool)
            items.remove(victim)
            removed.append(victim["UNIQUEID"])
        scene[key] = items
    if removed:
        gone = {str(i) for i in removed}
        scene["waypointDataList"] = [w for w in scene.get("waypointDataList", [])
                                     if str(w.get("pathName", "")).split(" - ")[-1] not in gone]
        Path(scenario_path).write_text(json.dumps(scene, ensure_ascii=False, indent=2))
    return len(removed)


def remove_scenario(name):
    stem = Path(name).stem
    for path in (SCENARIO_DIR / name, SCENARIO_DIR / "EgoNetwork" / f"{stem}_MN.json"):
        if path.name.startswith("global_hard_"):
            path.unlink(missing_ok=True)


def rpc(script):
    subprocess.run(["docker", "exec", RPC_CONTAINER, "mkdir", "-p", "/tmp/morai_competition_code/runtime"], check=True)
    subprocess.run(["docker", "cp", f"{RPC_CODE}/.", f"{RPC_CONTAINER}:/tmp/morai_competition_code/runtime"],
                   check=True, capture_output=True)
    return subprocess.run(["docker", "exec", "-w", "/tmp/morai_competition_code", "-e",
                           "PYTHONPATH=/tmp/morai_competition_code", RPC_CONTAINER, "python", "-c", script],
                          check=True, capture_output=True, text=True, timeout=90).stdout


def load_scenario(name):
    boundary = PLAYER_LOG.stat().st_size if PLAYER_LOG.exists() else 0
    rpc("from runtime.morai_rpc import MoraiRPC\n"
        f"MoraiRPC().call('scenario.Scenario','LoadMoraiScenario',{{'value':{name!r}}},timeout=30)")
    marker = f"The scenario file has been Load in [{name}] Complete"
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        if PLAYER_LOG.exists():
            with PLAYER_LOG.open("rb") as stream:
                stream.seek(boundary if PLAYER_LOG.stat().st_size >= boundary else 0)
                if marker in stream.read().decode(errors="replace"):
                    return
        time.sleep(.2)
    raise RuntimeError("MORAI 시나리오 로드 완료 로그 없음: " + name)


def add_tunnel_obstacles(scenario_path, config, rng):
    """Append the tunnel-zone static objects of a competition-mode layout to this scene."""
    comp = dict(config, mode=None, events=10, start=204, end=1990, keepTraffic=False,
                kinds=list(STATIC_KINDS) + ["crossing"], missionPlan="competition-signals-tunnel-v2",
                missionCounts={"static": rng.randint(4, 10), "dynamic": 1}, seed=config["seed"] + "-tunnel")
    comp = {k: v for k, v in comp.items() if k not in ("mode", "theme", "highway_npcs", "tunnel_objects")}
    for attempt in range(6):
        try:
            generated = http(STUDIO + "/api/generate", {"config": comp}, timeout=120)
            break
        except urllib.error.HTTPError:
            comp["seed"] += "x"
    else:
        return []
    tunnel_ids = {a["id"] for a in generated["actors"] if a.get("missionZone") == "static"}
    objects = [o for o in generated["scene"]["objectList"] if o["UNIQUEID"] in tunnel_ids]
    scene = json.loads(Path(scenario_path).read_text())
    used = {o["UNIQUEID"] for key in ("objectList", "pedestrianList", "vehicleList") for o in scene.get(key, [])}
    added = []
    for o in objects:
        new_id = 30000 + len(added)
        while new_id in used:
            new_id += 1
        used.add(new_id)
        path = next((w for w in generated["scene"]["waypointDataList"] if w["pathName"] == f"ObstaclePath - {o['UNIQUEID']}"), None)
        o = dict(o, UNIQUEID=new_id)
        scene["objectList"].append(o)
        scene.setdefault("waypointDataList", []).append(dict(path or {"waypointData": []}, pathName=f"ObstaclePath - {new_id}"))
        added.append(dict(id=new_id, data_id=o["DataID"], x=round(o["pos"]["x"], 1), y=round(o["pos"]["y"], 1)))
    Path(scenario_path).write_text(json.dumps(scene, ensure_ascii=False, indent=2))
    return added



def path_points(paths, unique_id):
    """World xy of the waypoints of actor `unique_id` (studio pathName "... - <id>").
    Waypoints are stored in Unity axes: world (x, y) = (-pos.x, -pos.z), pos.y is height."""
    out = []
    for w in paths:
        if unique_id is None or str(w.get("pathName", "")).endswith(f" - {unique_id}"):
            for q in w.get("waypointData", []) or []:
                qp = q.get("pos") or q.get("position") or q
                try:
                    out.append((-float(qp["x"]), -float(qp["z"])))
                except (KeyError, TypeError, ValueError):
                    pass
    return out


def add_dynamic_zone_mix(scenario_path, config, rng):
    """Competition mode puts only crossing pedestrians in its dynamic mission (studio s
    408-499 m). Mix many static obstacles (every model family) and shoulder walkers into
    that same stretch (user 2026-10-05). Generated as a free box layout on the stretch;
    anything within 3 m of an existing actor is skipped."""
    # 408-499 alone fits ~8 objects + 5 walkers; 380-530 stays inside the planner's
    # obstacle section (s 245-545). Gap >= 12 m: with 4 m the studio put in-lane
    # objects every 5 m (a 4.6 m car cannot slalom that; auto run 151 stuck).
    mix_start = round(rng.uniform(260., 360.))
    box = dict(config, events=rng.randint(10, 30), start=mix_start, end=min(530, mix_start + round(rng.uniform(120., 270.))),
               gap=max(12, config.get("gap", 12)),
               keepTraffic=False,
               kinds=rng.sample(STATIC_KINDS, rng.randint(2, len(STATIC_KINDS))) + ["shoulder"],
               missionPlan=None, missionCounts=None, seed=config["seed"] + "-dynmix")
    box = {k: v for k, v in box.items() if k not in ("mode", "theme", "highway_npcs", "tunnel_objects", "dynamic_mix")}
    for attempt in range(6):
        try:
            generated = http(STUDIO + "/api/generate", {"config": box}, timeout=120)
            break
        except urllib.error.HTTPError:
            box["seed"] += "x"
            box["events"] = max(6, box["events"] - 2)
    else:
        return []
    scene = json.loads(Path(scenario_path).read_text())
    def pos(o):
        return (o.get("pos") or (o.get("initPosition") or {}).get("pos") or {})
    existing = [(float(pos(o)["x"]), float(pos(o)["y"])) for key in ("objectList", "pedestrianList")
                for o in scene.get(key, []) if pos(o)]
    # Keep the crosswalks clear: crossing people wait in the lane there, and a static
    # object beside them closed the road (auto runs 151-153, same spot each time).
    crossings = [(float(pos(o)["x"]), float(pos(o)["y"])) for o in scene.get("pedestrianList", []) if pos(o)]
    # (Waypoints were read as (pos.x, pos.y) before - Unity axes, so these never matched.)
    crossings += path_points([w for w in scene.get("waypointDataList", []) if "Ped" in str(w.get("pathName", ""))], None)
    used = {o["UNIQUEID"] for key in ("objectList", "pedestrianList", "vehicleList") for o in scene.get(key, [])}
    paths = generated["scene"].get("waypointDataList", [])
    added = []
    for key in ("objectList", "pedestrianList"):
        for o in generated["scene"].get(key, []):
            p = pos(o)
            if not p:
                continue
            xy = (float(p["x"]), float(p["y"]))
            if any(math.hypot(xy[0] - a, xy[1] - b) < 3. for a, b in existing):
                continue
            if key == "objectList" and any(math.hypot(xy[0] - a, xy[1] - b) < 12. for a, b in crossings):
                continue
            if key == "pedestrianList":
                # A shoulder walker that walks past / stops beside a static object stands
                # in the only gap around it, and we never squeeze past people: auto run
                # 198 waited 70 s for good. Its whole walk keeps 10 m from every object.
                corners = [xy] + path_points(paths, o["UNIQUEID"])
                walk = [(a0 + (a1 - a0) * t / 8., b0 + (b1 - b0) * t / 8.)
                        for (a0, b0), (a1, b1) in zip(corners, corners[1:] + corners[-1:]) for t in range(8)]
                statics = [(float(pos(q)["x"]), float(pos(q)["y"])) for q in scene.get("objectList", []) if pos(q)]
                if any(math.hypot(wx - a, wy - b) < 10. for wx, wy in walk for a, b in statics):
                    continue
            new_id = 40000 + len(added)
            while new_id in used:
                new_id += 1
            used.add(new_id)
            old_id = o["UNIQUEID"]
            scene.setdefault(key, []).append(dict(o, UNIQUEID=new_id))
            for w in paths:
                name = str(w.get("pathName", ""))
                if name.endswith(f" - {old_id}"):
                    scene.setdefault("waypointDataList", []).append(dict(w, pathName=name[: -len(str(old_id))] + str(new_id)))
            existing.append(xy)
            added.append(dict(id=new_id, kind="pedestrian" if key == "pedestrianList" else "static",
                              data_id=o.get("DataID"), x=round(xy[0], 1), y=round(xy[1], 1)))
    Path(scenario_path).write_text(json.dumps(scene, ensure_ascii=False, indent=2))
    return added


def slalom_tunnel_obstacles(scenario_path, rng):
    """Tunnel obstacles alternate between our lane and the left lane (S-shaped weave) instead of
    sitting on one side (user 2026-10-05). Re-spaced 10-16 m apart, 0.6-1.4 m off the
    route; extra copies of the scenario's own tunnel objects fill in to 4-8."""
    import numpy as np
    tm = json.loads((ASSETS / "tunnel_wall_map.json").read_text())
    origin = np.asarray(tm["origin_xy"], dtype=float)
    heading = math.radians(tm["axis_heading_deg"])
    axis, normal = np.array([math.cos(heading), math.sin(heading)]), np.array([-math.sin(heading), math.cos(heading)])
    start, end = float(tm["longitudinal_min_m"]), float(tm["longitudinal_max_m"])
    route = highway_lanes()[0]
    ra, rl = (route - origin) @ axis, (route - origin) @ normal
    inside = (ra >= start - 5.) & (ra <= end + 5.) & (np.abs(rl) < 12.)
    order = np.argsort(ra[inside])
    ra_in, rl_in = ra[inside][order], rl[inside][order]
    scene = json.loads(Path(scenario_path).read_text())
    tunnel = []
    for o in scene.get("objectList", []):
        q = np.array([float(o["pos"]["x"]), float(o["pos"]["y"])]) - origin
        if start - 2. <= q @ axis <= end + 2. and abs(q @ normal) < 9.:
            tunnel.append(o)
    if not tunnel:
        return []
    z = float(np.mean([float(o["pos"]["z"]) for o in tunnel]))
    # Slalom pieces up to car size: a 12 m bus is a wall, not a slalom gate (run 178).
    info = studio_model_info()
    small = [o for o in tunnel if info.get(o.get("DataID"), ("", 1., ""))[1] <= 3.0]
    for o in tunnel:
        if o not in small:
            scene["objectList"].remove(o)
    tunnel = small
    if not tunnel:
        Path(scenario_path).write_text(json.dumps(scene, ensure_ascii=False, indent=2))
        return []
    used = {o["UNIQUEID"] for key in ("objectList", "pedestrianList", "vehicleList") for o in scene.get(key, [])}
    # Last obstacle >= 35 m before the exit: a bus 20 m before it left 14 m to steer
    # back to the route and the car stuck beside it (auto run 178).
    stations, a = [], start + rng.uniform(15., 30.)
    while a < end - 35. and len(stations) < 8:
        stations.append(a)
        a += rng.uniform(10., 16.)
    if len(stations) < 4:
        return []
    side = rng.choice((-1., 1.))
    placed = []
    for k, a in enumerate(stations):
        if k < len(tunnel):
            o = tunnel[k]
        else:
            o = json.loads(json.dumps(tunnel[k % len(tunnel)]))
            new_id = 30100 + k
            while new_id in used:
                new_id += 1
            used.add(new_id)
            o["UNIQUEID"] = new_id
            scene["objectList"].append(o)
            scene.setdefault("waypointDataList", []).append({"pathName": f"ObstaclePath - {new_id}", "waypointData": []})
        # Alternate between our (right) lane and the left lane (user 2026-10-07: "why only
        # one lane?"); the tunnel allows lane use freely. Left lane centre is +3.5 m.
        lateral = float(np.interp(a, ra_in, rl_in)) + (3.5 + rng.uniform(-.9, .9) if side > 0 else rng.uniform(-1.4, 1.4))
        xy = origin + a * axis + lateral * normal
        o["pos"] = {"x": float(xy[0]), "y": float(xy[1]), "z": z, "_x": f"{xy[0]:.3f}", "_y": f"{xy[1]:.3f}", "_z": f"{z:.3f}"}
        placed.append(dict(id=o["UNIQUEID"], data_id=o.get("DataID"), along=round(a, 1), lateral=round(lateral, 2)))
        side = -side
    for o in tunnel[len(stations):]:          # surplus objects would break the S: drop them
        scene["objectList"].remove(o)
    Path(scenario_path).write_text(json.dumps(scene, ensure_ascii=False, indent=2))
    return placed

# ---------------------------------------------------------------- highway NPC traffic
STUDIO_TEMPLATE = Path.home() / "morai-scenario-studio/data/Scenario/morai_hard_09.json"
_HIGHWAY = None


def highway_lanes():
    """(route xy, {link: points}, nearest-link lookup) for the highway section, numpy only."""
    global _HIGHWAY
    if _HIGHWAY is None:
        import ast
        import numpy as np
        links = {l["idx"]: l for l in json.loads((ASSETS / "link_set.json").read_text())}
        pts = {}
        for k, l in links.items():
            raw = l["points"] if isinstance(l["points"], list) else ast.literal_eval(l["points"])
            pts[k] = np.asarray(raw, dtype=float)[:, :3]
        route = np.asarray([[p["x"], p["y"]] for p in json.loads(
            (ASSETS / "global_path.json").read_text())["waypoints"]])
        names = list(pts)
        stacked = np.vstack([pts[k][:, :2] for k in names])
        owner = np.concatenate([[i] * len(pts[k]) for i, k in enumerate(names)])
        _HIGHWAY = (route, links, pts, names, stacked, owner)
    return _HIGHWAY


def lane_point(route_idx, side):
    """(link id, point index, xyz, heading deg) of our lane (side 0) or the next lane (+1 left / -1 right)."""
    import numpy as np
    route, links, pts, names, stacked, owner = highway_lanes()
    j = int(np.argmin(np.linalg.norm(stacked - route[route_idx], axis=1)))
    link = names[owner[j]]
    if side:
        key = "left_lane_change_dst_link_idx" if side > 0 else "right_lane_change_dst_link_idx"
        other = links[link].get(key)
        if other in (None, "None") or other not in pts:
            return None
        link = other
    p = pts[link]
    k = int(np.argmin(np.linalg.norm(p[:, :2] - route[route_idx], axis=1)))
    k = min(k, len(p) - 2)
    heading = math.degrees(math.atan2(p[k + 1, 1] - p[k, 1], p[k + 1, 0] - p[k, 0]))
    return link, k, p[k], heading


TUNNEL_GUARD_IDX = 3000   # 3400 let NPCs take link 240, which forks off at the tunnel mouth (run 155)

def off_route_end(link):
    """First link >= 30 m off our route reachable from `link` (breadth-first, every
    branch): an NPC that ends on our route parks there (run 102: two stopped at the
    tunnel mouth). Branches through the tunnel are never used: an NPC cannot pass
    its obstacles, stops inside and we hit it (auto run 124)."""
    import numpy as np
    from collections import deque
    route, links, pts, names, stacked, owner = highway_lanes()
    successors = {}
    for k, l in links.items():
        successors.setdefault(l.get("from_node_idx"), []).append(k)
    queue, seen = deque([(link, 0)]), {link}
    while queue:
        current, depth = queue.popleft()
        if current not in pts:
            continue
        p = pts[current]
        dist = np.linalg.norm(route[None, :, :] - p[::5, None, :2], axis=2)
        d = np.min(dist, axis=1)
        on_route = d < 5.
        if on_route.any() and int(np.argmin(dist[on_route], axis=1).max()) >= TUNNEL_GUARD_IDX:
            continue
        if d.min() > 30.:
            return current, len(p) // 2
        if depth >= 40:
            continue
        for k in successors.get(links[current].get("to_node_idx"), []):
            if k not in seen:
                seen.add(k)
                queue.append((k, depth + 1))
    return None


def move_traffic_ends(scenario_path):
    """Scenario traffic that ends next to our route parks there: two cars stopped at the
    highway exit merge 1.4 m from our lane and held us for good (auto run 109). Their
    end points move to the first successor link >= 30 m off our route."""
    import numpy as np
    route, links, pts, names, stacked, owner = highway_lanes()
    scene = json.loads(Path(scenario_path).read_text())
    moved = 0
    for v in scene.get("vehicleList", []):
        end = v.get("endPointLinkInfo") or {}
        link = end.get("linkIdx")
        if link not in pts:
            continue
        p = pts[link][min(int(end.get("pointIdx", 0)), len(pts[link]) - 1)]
        if float(np.min(np.linalg.norm(route - p[:2], axis=1))) >= 15.:
            continue
        away = off_route_end(link)
        if away is not None:
            v["endPointLinkInfo"] = {"linkIdx": away[0], "pointIdx": away[1]}
            moved += 1
    Path(scenario_path).write_text(json.dumps(scene, ensure_ascii=False, indent=2))
    return moved


def add_highway_npcs(scenario_path, rng):
    """Overtaking cars (adjacent lane, start as we pass at 70-100 km/h) and slow cars
    (our lane, start 80 m ahead of us at 30-50 km/h) on the highway (user 2026-10-05)."""
    template = next(v for v in json.loads(STUDIO_TEMPLATE.read_text())["vehicleList"] if "startPointLinkInfo" in v)
    scene = json.loads(Path(scenario_path).read_text())
    added = []
    wanted = rng.randint(1, 5)
    for n in range(20):                   # spawns past the last exit are skipped: retry
        if len(added) >= wanted:
            break
        overtake = rng.random() < .6
        start_idx = rng.randint(2350, 3000)
        here = lane_point(start_idx, (1 if rng.random() < .8 else -1) if overtake else 0)
        end = lane_point(3220, 1 if overtake else 0)
        if here is None or end is None:
            continue
        # Leave by the exit before the tunnel: past idx ~2950 the only way on is the
        # tunnel, which an NPC cannot pass (auto run 124) - those spawns are skipped.
        away = off_route_end(here[0])
        if away is None:
            # No exit avoids the tunnel mouth: stop in the next lane just before the
            # highway end (exit speed limit there anyway), never on our own lane.
            side_end = lane_point(2980, 1) or lane_point(2980, -1)
            if side_end is None or start_idx > 2980 - 60:
                # The end must lie well ahead: spawned at idx 2949 with this end 31 idx
                # (~15 m) ahead on the same link, an NPC looped round the whole map and
                # met us inside the tunnel (auto run 176).
                continue
            import numpy as np
            route = highway_lanes()[0]
            if float(np.min(np.linalg.norm(route - np.asarray(side_end[2][:2]), axis=1))) < 3.:
                continue
            away = (side_end[0], side_end[1])
        end = (away[0], away[1])
        link, k, xyz, heading = here
        if end[0] == link and int(end[1]) <= int(k) + 20:
            continue                      # same link, end behind/at the spawn: MORAI would loop the map
        if not overtake and any(a["kind"] == "slow" for a in added):
            continue                      # one slow car per scene
        # Slow cars stay >= 50 km/h: 31-40 km/h leaders held us at 25-30 km/h behind them
        # (runs 96-98); the highway floor is 50 (user rule).
        kmh = rng.uniform(70., 100.) if overtake else rng.uniform(50., 65.)
        v = json.loads(json.dumps(template))
        v["UNIQUEID"] = 20000 + n
        v["initPosition"]["pos"] = {"x": float(xyz[0]), "y": float(xyz[1]), "z": float(xyz[2]) + .3,
                                    "_x": f"{xyz[0]:.3f}", "_y": f"{xyz[1]:.3f}", "_z": f"{xyz[2] + .3:.3f}"}
        v["initPosition"]["rot"] = {"roll": "0.000", "pitch": "0.000", "yaw": f"{heading:.3f}"}
        v["initPosition"]["initLink"] = link
        v["initPosition"]["initLinkRatio"] = 0
        # Spawn speed (MORAI saves both fields, km/h). Run 112 showed MORAI still starts
        # them from 0, hence the long activation distances below.
        v["initPosition"].setdefault("vehicleSaveData", {})["velocity"] = round(kmh, 1)
        v["startPointLinkInfo"] = {"linkIdx": link, "pointIdx": k}
        v["currentLinkInfo"] = {"linkIdx": link, "pointIdx": k}
        v["endPointLinkInfo"] = {"linkIdx": end[0], "pointIdx": end[1]}
        v["desiredVelocity"] = v["constantVelocity"] = round(kmh, 1)
        v["currentVelocity"] = v["viewCurrentVelocity"] = round(kmh, 1)
        # Overtakers never change lanes: still accelerating from 0 (MORAI ignores the
        # spawn speed) one ran beside us at 25-34 km/h and merged into our side (run 112).
        v["isLaneChange"] = False if overtake else rng.random() < .5
        v["isCloseLoop"] = False
        v["spawnPointUniqueID"] = v["destinationUniqueID"] = -1
        # Start well before we arrive: parked "overtakers" (15 m trigger) sat in the next
        # lane and read as static obstacles, holding the highway under 50 km/h (runs 89-97).
        # Up to speed before we see them: at 80 m a slow car was still standing (0 km/h) when
        # our LiDAR found it and we braked below 50 (run 105).
        v["activeDistance"] = 400. if overtake else 600.   # slow car still at 41-46 km/h from standstill at 250 m (run 128)
        scene.setdefault("vehicleList", []).append(v)
        added.append(dict(kind="overtake" if overtake else "slow", route_idx=start_idx, link=link,
                          kmh=round(kmh, 1), lane_change=v["isLaneChange"]))
    Path(scenario_path).write_text(json.dumps(scene, ensure_ascii=False, indent=2))
    return added


# ---------------------------------------------------------------- MORAI network
def missing_udp_ports():
    """Host ports of the single-PC UDP preset that the simulator does not hold."""
    import competition_profiles          # same request student_v3_follow_claude.sh verifies
    request = competition_profiles.network_request("single_pc")
    wanted = {f"{v['NETWORK_ITEM_TYPE_HOST_IP']}:{v['NETWORK_ITEM_TYPE_HOST_PORT']}"
              for v in ({x["type"]: x["value"] for x in item["items"]} for item in request["protocol_settings"]["items"])}
    owned = {line.split()[3] for line in subprocess.run(["ss", "-Hlunp"], capture_output=True, text=True).stdout.splitlines()
             if "Simulator.x86" in line}
    return sorted(wanted - owned)


def gui_click(window, dx, dy, pause=1.):
    # Offsets are relative to xdotool's reported window position (GNOME frame shifts it by -10,-36).
    env = dict(os.environ, DISPLAY=os.environ.get("DISPLAY", ":0"))
    geometry = subprocess.run(["xdotool", "getwindowgeometry", "--shell", window], capture_output=True,
                              text=True, env=env, check=True).stdout
    values = dict(line.split("=") for line in geometry.split())
    subprocess.run(["xdotool", "mousemove", str(int(values["X"]) + dx), str(int(values["Y"]) + dy), "click", "1"],
                   env=env, check=True)
    time.sleep(pause)


def load_network_gui():
    """Edit > Network Settings > Load > 2026_molit_comp_network_single_pc.json > Load > OK > close."""
    env = dict(os.environ, DISPLAY=os.environ.get("DISPLAY", ":0"))
    window = subprocess.run(["xdotool", "search", "--name", "^Simulator$"], capture_output=True, text=True,
                            env=env).stdout.split()[0]
    subprocess.run(["xdotool", "windowactivate", "--sync", window], env=env)
    time.sleep(1)
    for dx, dy in ((20, -36), (53, 59), (490, 497), (300, 238), (403, 498), (630, 401)):
        gui_click(window, dx, dy, 1.5)
    if missing_udp_ports():
        gui_click(window, 825, 497, 3.)      # Connect (only while still disconnected)
    gui_click(window, 1042, 126)              # close the Network Settings window


def ensure_network():
    missing = missing_udp_ports()
    if not missing:
        return
    log("MORAI UDP network missing - loading single-PC preset", missing=missing)
    load_network_gui()
    time.sleep(2)
    missing = missing_udp_ports()
    if missing:
        raise RuntimeError("MORAI 네트워크 로드 실패: " + " ".join(missing))


# ---------------------------------------------------------------- driving
def job():
    # The dashboard answers slowly under heavy disk I/O; a slow reply must not abort a lap.
    for attempt in range(5):
        try:
            return http(WEB + "/api/status", timeout=60)["job"]
        except Exception:
            if attempt == 4:
                raise
            time.sleep(5)


def wait_job_idle(timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not job().get("running"):
            return True
        time.sleep(2)
    return False


def stop_job():
    try:
        http(WEB + "/api/stop", {}, timeout=30)
    except Exception as error:
        log("stop request failed", error=str(error))
    return wait_job_idle(150)


def loc_status(sid):
    runs = sorted((DATA / "autodrive/mpc_runs" / sid).glob("*/loc_status.json"), key=lambda p: p.stat().st_mtime)
    if not runs:
        return None
    try:
        return json.loads(runs[-1].read_text())
    except (OSError, ValueError):
        return None


ORPHAN_PATTERNS = ("autodrive/lidar_vehicle_detection.py", "autodrive/semantic_fusion.py",
                   "student_v3_claude/gui.py", "autodrive/localization_monitor.py")
# The ESKF launch is left running between laps on purpose (the next lap reuses it), but
# after a start-up failure ("ESKF not tracking GPS ... restarting localization") the
# leftover one kept the retry from publishing /localization/ego_status (runs 230/231, 250).
LOCALIZATION_PATTERNS = ("gps_odometry_eskf.launch", "eskf/eskf_node.py", "ego_status_from_eskf.py",
                         "pure_odometry", "sensor_tf_from_config")


def kill_orphans(localization=False):
    """Between laps nothing of the driving stack may run. A PointPillars detector left by a
    force-stopped lap kept publishing for 4.5 h and halved the live detector's rate
    (highway stops on "sensor stale", 2026-10-05). localization=True also stops the ESKF
    (start-up failure retry only)."""
    patterns = ORPHAN_PATTERNS + (LOCALIZATION_PATTERNS if localization else ())
    out = subprocess.run(["ps", "-eo", "pid=,args="], capture_output=True, text=True).stdout
    for line in out.splitlines():
        pid, _, args = line.strip().partition(" ")
        if any(p in args for p in patterns) and "auto_collect" not in args and "/bin/bash" not in args:
            try:
                os.kill(int(pid), signal.SIGTERM)
                log("killed orphan driving process", pid=pid, args=args[:120])
            except (ProcessLookupError, ValueError, PermissionError):
                pass


def drive_one(name, config, run_index, route_n):
    if not job().get("running"):
        kill_orphans()
        time.sleep(1)
    http(WEB + "/api/start", {"action": "autodrive_collect", "options": {"minutes": LAP_MINUTES}}, timeout=30)
    started = time.monotonic()
    current = job()
    sid = str(current["command"][-1])        # web_app puts the session id last
    log("drive started", sid=sid, scenario=name)
    # MORAI ground truth (rear-axle pose + 4 wheel positions) for the self-judge only;
    # never an input to the driving stack (rule: judge-only data).
    truth_path = DATA / "autodrive/truth" / f"{sid}.jsonl"
    truth = None
    if (JUDGE_DIR / "record_truth.py").is_file():
        truth_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            truth = subprocess.Popen(["nice", "-n", "19", sys.executable, str(JUDGE_DIR / "record_truth.py"),
                                      "--out", str(truth_path), "--seconds", "1200"],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        except OSError:
            truth = None
    write_status(phase="driving", sid=sid, scenario=name, run=run_index, progress=0.)
    copied = False
    progress, prev_idx, prev_t, last_progress_t, moving = 0, None, time.monotonic(), time.monotonic(), False
    seen_end = seen_mid = False
    outcome = "time_limit"
    collisions, ego_offset = [], 0
    while True:
        time.sleep(1)
        if not copied and (RAW / sid / "session.json").exists():
            (RAW / sid / ".auto_hold").touch()     # uploader waits for the collision verdict
            # Ground truth of every placed obstacle/pedestrian for labelling, uploaded with the session.
            target = RAW / sid / "scenario"
            target.mkdir(exist_ok=True)
            shutil.copy2(SCENARIO_DIR / name, target / name)
            (target / "generator_config.json").write_text(json.dumps(
                dict(config=config, scenario=name, run=run_index), ensure_ascii=False, indent=2))
            copied = True
        # Collisions do not end or discard the lap: finalize_session.py writes their time
        # windows to collisions.json (existing Drive format) and training masks those moments.
        hit, ego_offset = scan_collisions(sid, ego_offset)
        if hit:
            collisions.extend(hit)
        state = job()
        if not state.get("running"):
            outcome = "time_limit" if state.get("exit_code") in (0, None) else f"exit_{state.get('exit_code')}"
            break
        if should_stop():
            outcome = "stopped_by_user"
            stop_job()
            break
        status = loc_status(sid)
        idx = ((status or {}).get("path") or {}).get("nearest_idx")
        if idx is not None:
            idx = int(idx)
            # Lap = reached the route end, then wrapped to its start. Summed steps
            # under-counted (large jumps skipped): runs 64/66 finished at "0.66".
            # Checkpoint lap: past the tunnel entry (idx ~3500 of 4392), then back near
            # the start line. Summed progress under-counts (run 72: "0.45" after a lap).
            if route_n * .3 < idx < route_n * .6:
                seen_mid = True               # the start sits at the route end: require mid-lap first
            if seen_mid and idx > route_n * .8:
                seen_end = True
            elif seen_end and idx < route_n * .05:
                progress = route_n
            now_t = time.monotonic()
            if prev_idx is not None:
                step = (idx - prev_idx) % route_n
                # ~0.5 m points; a jump beyond 60 points/s (~108 km/h) is a nearest-point
                # jump. Scaled by the real poll interval: under load the poll took several
                # seconds, 89 km/h highway steps exceeded the fixed 120 and a moving car was
                # stopped as "stuck" in the tunnel (auto run 213).
                if 0 < step < max(120., 60. * (now_t - prev_t)):
                    progress += step
                    if step > 2:
                        last_progress_t, moving = now_t, True
            prev_idx, prev_t = idx, now_t
        write_status(progress=round(progress / route_n, 3), elapsed_s=round(time.monotonic() - started))
        if progress >= route_n * .97:
            outcome = "lap_complete"
            stop_job()
            break
        if moving and time.monotonic() - last_progress_t > STUCK_S:
            outcome = "stuck"
            stop_job()
            break
    if not copied:
        log("scenario not copied (no session folder)", sid=sid)
    if truth is not None:
        truth.terminate()
        try:
            truth.wait(5)
        except subprocess.TimeoutExpired:
            truth.kill()
    try:
        if truth_path.exists() and (RAW / sid).is_dir():
            shutil.copy(truth_path, RAW / sid / "truth_raw.jsonl")
    except OSError as error:
        log("truth copy failed", sid=sid, error=str(error))
    hit, ego_offset = scan_collisions(sid, ego_offset)
    collisions.extend(hit or [])
    if (JUDGE_DIR / "judge.py").is_file():
        try:
            verdict = subprocess.run(["nice", "-n", "19", sys.executable, str(JUDGE_DIR / "judge.py"), sid, "--quiet"],
                                     capture_output=True, text=True, timeout=120, cwd=str(JUDGE_DIR))
            log("judge", sid=sid, line=(verdict.stdout.strip().splitlines() or [""])[-1][:300])
        except Exception as error:
            log("judge failed", sid=sid, error=str(error)[:200])
    result = dict(sid=sid, outcome=outcome, lap_fraction=round(progress / route_n, 3),
                  duration_s=round(time.monotonic() - started), collision_msgs=len(collisions),
                  collided_with=sorted(set(collisions))[:10])
    release_session(sid)
    return result


_COLLISION_STATE = {}


def scan_collisions(sid, offset):
    """Read new /CollisionData lines that the ego recorder appends to ego_raw.jsonl.
    MORAI lists the ego itself (id 0 / type -1) and also objects colliding with
    other things (pedestrians stuck in scenery, listed from t=0 at 400 m; auto run
    144). Ours: an object whose listing starts (gap > 0.5 s) within 3.5 m of us."""
    path = RAW / sid / "ego_raw.jsonl"
    hits = []
    if not path.exists():
        return hits, offset
    state = _COLLISION_STATE.setdefault(sid, dict(last_ego=None, last_seen={}, ours={}))
    with path.open("rb") as stream:
        stream.seek(offset)
        chunk = stream.read()
    end = chunk.rfind(b"\n") + 1
    for line in chunk[:end].splitlines():
        # The recorder writes '"type": "ego"' (with a space); the old b'"type":"ego"'
        # never matched, so the distance filter never ran.
        if b'"ego"' in line and b'"collision"' not in line:
            try:
                rec = json.loads(line)
                if rec.get("type") == "ego":
                    state["last_ego"] = (rec["x"], rec["y"])
            except (ValueError, KeyError):
                pass
            continue
        if b'"collision"' not in line:
            continue
        try:
            record = json.loads(line)
        except ValueError:
            continue
        if record.get("type") != "collision":
            continue
        t_us = int(record.get("t_us", 0))
        for o in record.get("objects", []):
            if o.get("type") == -1 or o.get("id") == 0:
                continue
            key = o.get("id")
            if key not in state["last_seen"] or t_us - state["last_seen"][key] > 500_000:
                ego = state["last_ego"]
                state["ours"][key] = ego is not None and math.hypot(o.get("x", 0.) - ego[0], o.get("y", 0.) - ego[1]) < 3.5
            state["last_seen"][key] = t_us
            if state["ours"][key]:
                hits.append(str(o.get("name")))
    return hits, offset + end

def mark_finished(session):
    """A lap stopped hard can leave session.json without finished_at although the
    frames and finalize audit are complete (run 52); the uploader skips it forever."""
    path = session / "session.json"
    try:
        meta = json.loads(path.read_text())
    except (OSError, ValueError):
        return
    if meta.get("finished_at") or "audit" not in meta:
        return
    meta["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S+00:00", time.gmtime())
    meta["finished_note"] = "set by auto_collect: driver stopped before writing finished_at"
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(meta, indent=2))
    os.replace(tmp, path)


def release_stale_holds():
    """Between laps nothing is recording: release holds left by an aborted lap (run 48)."""
    for session in RAW.iterdir():
        if session.is_dir() and (session / ".auto_hold").exists():
            mark_finished(session)
            (session / ".auto_hold").unlink(missing_ok=True)


def release_session(sid):
    """Release the session to the uploader once finalize_session.py has written collisions.json."""
    session = RAW / sid
    deadline = time.monotonic() + 120
    while time.monotonic() < deadline:
        try:
            meta = json.loads((session / "session.json").read_text())
            if meta.get("finished_at"):
                break
        except (OSError, ValueError):
            pass
        time.sleep(2)
    else:
        mark_finished(session)
    (session / ".auto_hold").unlink(missing_ok=True)


def prune_debug_bags(keep=8, low_gb=20.):
    """Per-run rosbags (raw LiDAR debug, 0.15-0.7 GB) live on the internal disk; Drive
    quota errors stopped debug_backup.sh and they filled it (runs 132/133 could not start
    recording, 2026-10-05). When that disk runs low, delete bags of all but the newest
    `keep` runs. Training data (sessions on RAW) and decision logs are never touched."""
    root = DATA / "autodrive/mpc_runs"
    if shutil.disk_usage(root).free / 1e9 >= low_gb:
        return
    runs = sorted((p for p in root.iterdir() if p.is_dir() and p.name.isdecimal()), key=lambda p: int(p.name))
    removed = 0
    for run in runs[:-keep]:
        for bag in list(run.glob("*/*.bag")) + list(run.glob("*/*.bag.active")):
            try:
                bag.unlink()
                removed += 1
            except OSError:
                pass
    if removed:
        log("debug bags pruned (internal disk low)", removed=removed,
            free_gb=round(shutil.disk_usage(root).free / 1e9, 1))


def wait_for_disk():
    while not should_stop():
        prune_debug_bags()
        # Both disks: sessions go to RAW (external), run logs/bags to the internal one.
        free = min(shutil.disk_usage(RAW).free, shutil.disk_usage(DATA / "autodrive").free) / 1e9
        if free >= MIN_FREE_GB:
            return
        write_status(phase="waiting_disk", free_gb=round(free, 1))
        time.sleep(30)


def main():
    global stopping

    def on_signal(signum, frame):
        global stopping
        stopping = True
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    STATE.mkdir(parents=True, exist_ok=True)
    STOP_FILE.unlink(missing_ok=True)
    route_n = route_length()
    rng = random.Random()
    limit = int(os.environ.get("AUTO_COLLECT_RUNS", "0"))      # 0 = forever
    run_index, failures = 0, 0
    log("auto collection started", route_points=route_n, minutes=LAP_MINUTES, limit=limit)
    if not wait_job_idle(5):
        raise SystemExit("다른 주행이 실행 중 — 끝난 뒤 다시 시작")
    while not should_stop() and (not limit or run_index < limit):
        wait_for_disk()
        if should_stop():
            break
        release_stale_holds()
        run_index += 1
        name = None
        try:
            write_status(phase="generating", run=run_index)
            name, config = make_scenario(rng, run_index)
            write_status(phase="loading", scenario=name)
            for attempt in range(3):
                try:
                    load_scenario(name)
                    break
                except Exception as error:
                    log("load failed", scenario=name, attempt=attempt + 1, error=str(error)[-300:])
                    time.sleep(5)
            else:
                raise RuntimeError("시나리오 로드 3회 실패")
            time.sleep(3)
            ensure_network()
            result = drive_one(name, config, run_index, route_n)
            if str(result.get("outcome", "")).startswith("exit_") and float(result.get("duration_s") or 0) < 90:
                # Start-up failure (run 123: ESKF not publishing after the vehicle reset):
                # reload and retry the same scenario once instead of discarding it.
                log("start-up failure; retrying the same scenario", outcome=result.get("outcome"))
                kill_orphans(localization=True)
                time.sleep(3)
                load_scenario(name)
                time.sleep(3)
                ensure_network()
                result = drive_one(name, config, run_index, route_n)
            failures = 0 if result["outcome"] in ("lap_complete", "time_limit", "stuck") else failures + 1
            result.update(run=run_index, scenario=name, config=config)
        except Exception as error:
            failures += 1
            result = dict(run=run_index, scenario=name, outcome="error", error=str(error)[-500:])
            if job().get("running"):
                stop_job()
        finally:
            if name:
                remove_scenario(name)
        with (STATE / "runs.jsonl").open("a") as out:
            out.write(json.dumps(dict(result, t=time.time()), ensure_ascii=False) + "\n")
        log("run finished", **{k: v for k, v in result.items() if k != "config"})
        write_status(phase="between_runs", last=result.get("outcome"), runs=run_index, failures=failures)
        if failures >= 3:
            write_status(phase="backoff", message="연속 3회 실패 — 2분 대기 후 재시도")
            for _ in range(120):
                if should_stop():
                    break
                time.sleep(1)
            failures = 0
        else:
            time.sleep(5)
        if RESTART_FILE.exists():
            # Reload code changes between laps only (an outside kill once cut a lap short).
            RESTART_FILE.unlink(missing_ok=True)
            log("restarting with updated code")
            os.execv(sys.executable, [sys.executable] + sys.argv)
    write_status(phase="stopped")
    log("auto collection stopped", runs=run_index)


if __name__ == "__main__":
    main()
