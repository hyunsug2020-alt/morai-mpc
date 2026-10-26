"""Shared surveyed intersection bounds for GUI checks and vehicle perception."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from stop_line import COMPETITION_STOP_NODE_IDS

APPROACH_M = 80.0
EXIT_M = 25.0
DETECTOR_PADDING_M = 10.0


def build_intersection_regions(stop_lines, route_junction, route_station,
                               node_ids=COMPETITION_STOP_NODE_IDS):
    """Retain HD-map stop projection and extend through the actual junction exit."""
    station = np.asarray(route_station, dtype=float)
    junction = np.asarray(route_junction, dtype=bool)
    if (station.ndim != 1 or len(station) < 2 or junction.shape != station.shape
            or not np.isfinite(station).all() or np.any(np.diff(station) < 0.)):
        raise ValueError("Invalid intersection route geometry")
    selected = [line for line in stop_lines if line.get("node_id") in node_ids]
    missing = set(node_ids)-{line["node_id"] for line in selected}
    if missing:
        raise ValueError("Requested intersection perception missing from HD map: " + ", ".join(sorted(missing)))
    entries = np.flatnonzero(junction[1:] & ~junction[:-1])+1
    regions = []
    for line in selected:
        route_index, stop_station = int(line["route_idx"]), float(line["station_m"])
        if not math.isfinite(stop_station) or not 0 <= route_index < len(station) or not len(entries):
            raise ValueError("Invalid surveyed intersection stop")
        entry = int(entries[np.argmin(np.abs(entries-route_index))])
        if abs(entry-route_index) > 12:
            raise ValueError("Surveyed stop does not match a route junction entry")
        end = entry
        while end < len(junction) and junction[end]:
            end += 1
        if end == len(junction):
            raise ValueError("Requested intersection has no surveyed route exit")
        regions.append(dict(node_id=str(line["node_id"]), line_id=str(line["id"]),
            stop_station_m=stop_station, start_station_m=max(0., stop_station-APPROACH_M),
            exit_station_m=float(station[end]), end_station_m=float(station[end])+EXIT_M,
            entry_route_idx=entry, exit_route_idx=end, entry_link_id=line.get("entry_link_id")))
    return sorted(regions, key=lambda region: region["start_station_m"])


def active_intersection(station_m, regions, padding_m=0.):
    """Return the active region; detector-only padding permits pose/warmup handoff."""
    try:
        station, padding = float(station_m), float(padding_m)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(station) or not math.isfinite(padding) or padding < 0.:
        return None
    return next((region for region in regions if
        region["start_station_m"]-padding-1e-8 <= station <= region["end_station_m"]+padding+1e-8), None)


def route_digest(route_xy):
    route = np.asarray(route_xy, dtype="<f8")
    if route.ndim != 2 or route.shape[1] != 2 or len(route) < 2 or not np.isfinite(route).all():
        raise ValueError("Invalid perception route")
    return hashlib.sha256(route.tobytes(order="C")).hexdigest()


def write_intersection_regions(path, route_xy, regions, node_ids=COMPETITION_STOP_NODE_IDS):
    payload = dict(schema=1, route_sha256=route_digest(route_xy),
                   node_ids=sorted(node_ids), approach_m=APPROACH_M, exit_m=EXIT_M, regions=regions)
    path = Path(path)
    temporary = path.with_name(path.name+".tmp")
    temporary.write_text(json.dumps(payload, indent=2))
    temporary.replace(path)
    return payload


def load_intersection_regions(path, route_xy, node_ids=COMPETITION_STOP_NODE_IDS):
    payload = json.loads(Path(path).read_text())
    if not isinstance(payload, dict):
        raise ValueError("Intersection perception configuration must be an object")
    if (payload.get("schema") != 1 or payload.get("route_sha256") != route_digest(route_xy)
            or set(payload.get("node_ids", ())) != set(node_ids)
            or payload.get("approach_m") != APPROACH_M or payload.get("exit_m") != EXIT_M):
        raise ValueError("Intersection perception configuration does not match route/rules")
    regions = payload.get("regions")
    if (not isinstance(regions, list) or not all(isinstance(r, dict) for r in regions)
            or {r.get("node_id") for r in regions} != set(node_ids)):
        raise ValueError("Intersection perception regions are incomplete")
    for region in regions:
        values = [float(region[key]) for key in ("start_station_m", "stop_station_m", "exit_station_m", "end_station_m")]
        if (not all(math.isfinite(value) for value in values) or not values[0] <= values[1] <= values[2] < values[3]
                or abs(values[0]-max(0., values[1]-APPROACH_M)) > 1e-6
                or abs(values[3]-values[2]-EXIT_M) > 1e-6
                or not 0 <= int(region["entry_route_idx"]) < int(region["exit_route_idx"]) < len(route_xy)):
            raise ValueError("Invalid intersection perception bounds")
    return regions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--links", type=Path, required=True)
    parser.add_argument("--route", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    # Map extraction belongs to HdMapLanes. The detector imports only the pure
    # functions above and never loads GUI/Student/model modules for this config.
    from gui import HdMapLanes
    route = np.asarray([[p["x"], p["y"]] for p in json.loads(args.route.read_text())["waypoints"]], dtype=float)
    lanes = HdMapLanes(args.links)
    lanes.set_route(route)
    regions = build_intersection_regions(lanes.stop_lines, lanes.route_junction, lanes.route_station)
    write_intersection_regions(args.output, route, regions)
    print(json.dumps(dict(intersection_perception=regions)), flush=True)


if __name__ == "__main__":
    main()
