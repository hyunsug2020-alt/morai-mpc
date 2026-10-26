#!/usr/bin/env python3
"""Build fixed tunnel walls from an empty recorded traversal and validate a separate capture."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from lidar_corridor import fit_road_plane
from tunnel_map import TunnelMap, _rotation, _dot, fit_wall


def capture_frames(path, scan_indices=None):
    data = np.load(path)
    # NPZ member access decompresses the entire array; do it once per capture.
    ego, xyz, offsets, stamps = (data[key] for key in ('ego', 'xyz', 'offsets', 'stamps'))
    data.close()
    timed_header = ego.shape[1] == 7 and np.median(ego[:, 1]) > 1e8
    pose_time = ego[:, 1] if timed_header else ego[:, 0]
    for scan in (range(len(stamps)) if scan_indices is None else scan_indices):
        stamp = stamps[scan]
        at = int(np.argmin(np.abs(pose_time-(stamp[1] if timed_header else stamp[0]))))
        row = ego[at]
        x, y, heading = (row[2], row[3], row[4]) if timed_header else (row[1], row[2], row[4])
        a, b = offsets[scan:scan+2]
        points = xyz[a:b].astype(float)+[1.05, 0., 1.35]
        near = points[(points[:, 0] > 2.) & (points[:, 0] < 15.) & (np.abs(points[:, 1]) < 3.)]
        floor = float(np.percentile(near[:, 2], 5)) if len(near) else -.4
        plane = fit_road_plane(points, floor)
        yield scan, points, dict(x=float(x), y=float(y), heading_deg=float(heading)), np.asarray(plane)


def build(recorded, route_file):
    waypoints = json.loads(Path(route_file).read_text())['waypoints']
    route = np.array([[p['x'], p['y']] for p in waypoints])
    station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    tree = cKDTree(route)
    observations = []
    for scan, points, ego, plane in capture_frames(recorded):
        _, idx = tree.query([ego['x'], ego['y']])
        if not 1879. <= station[idx] <= 2000.:
            continue
        height = points[:, 2]-(_dot(points[:, :2], plane[:2])+plane[2])
        structure = np.column_stack((points[:, :2], height))
        walls = [fit_wall(structure, side=side) for side in (1, -1)]
        if any(wall is None for wall in walls):
            continue
        left, right = walls
        width = left['offset']-right['offset']
        parallel = abs(math.degrees(math.atan(left['slope'])-math.atan(right['slope'])))
        if not 11.5 < width < 12.5 or parallel > .6:
            continue
        angle = ego['heading_deg']+math.degrees(math.atan((left['slope']+right['slope'])/2.))
        observations.append(dict(scan=scan, ego=ego, left=left, right=right,
                                 width=width, angle=angle, route_station_m=float(station[idx])))
    if len(observations) < 10:
        raise ValueError(f'insufficient supported two-wall scans: {len(observations)}')
    early = observations[:7]
    heading = float(np.median([o['angle'] for o in early]))
    origin = np.array([early[0]['ego']['x'], early[0]['ego']['y']])
    axis, normal = _rotation(heading).T
    offsets = []
    for o in early:
        lateral = (np.array([o['ego']['x'], o['ego']['y']])-origin) @ normal
        offsets.append([lateral+o[side]['offset'] for side in ('left', 'right')])
    left, right = np.median(offsets, axis=0)
    observed = []
    for o in observations:
        corrected_lateral = ((left-o['left']['offset'])+(right-o['right']['offset']))/2.
        old = np.array([o['ego']['x'], o['ego']['y']])
        correction = corrected_lateral-(old-origin) @ normal
        corrected_position = old+correction*normal
        scan_heading = heading-math.degrees(math.atan((o['left']['slope']+o['right']['slope'])/2.))
        for side in ('left', 'right'):
            world = o[side]['points'][:, :2] @ _rotation(scan_heading).T+corrected_position
            observed.extend(((world-origin) @ axis).tolist())
    # Endpoints are supported by observed wall returns, not extrapolated from the HD map.
    lower, upper = np.percentile(observed, [.5, 99.5])
    payload = dict(format='morai_fixed_tunnel_walls_v1', origin_xy=origin.tolist(),
        axis_heading_deg=heading, left_offset_m=float(left), right_offset_m=float(right),
        longitudinal_min_m=float(lower), longitudinal_max_m=float(upper),
        max_lateral_correction_m=.8, max_yaw_correction_deg=3., wall_filter_tolerance_m=.03,
        entry_anchor=dict(position=origin.tolist(), heading_deg=early[0]['ego']['heading_deg'],
                          scan_index=early[0]['scan'], route_station_m=early[0]['route_station_m'],
                          source='entry localized ego; GPS fused before blackout'),
        build=dict(recorded_capture=str(Path(recorded).resolve()), supported_scans=len(observations),
                   anchor_scans=len(early), measured_width_median_m=float(np.median([o['width'] for o in observations])),
                   width_p95_error_m=float(np.percentile(np.abs(np.array([o['width'] for o in observations])-(left-right)),95)),
                   measured_world_heading_range_deg=[min(o['angle'] for o in observations), max(o['angle'] for o in observations)]))
    payload['walls'] = {side: [(origin+axis*along+normal*offset).tolist() for along in (lower, upper)]
                        for side, offset in (('left', left), ('right', right))}
    return payload


def validate(model, capture):
    rows = []
    for scan, points, ego, plane in capture_frames(capture):
        if not model.active(ego):
            continue
        result = model.process(points, ego, plane)
        rows.append(dict(scan=scan, valid=result['valid'], reason=result['reason'],
                         wall_stats=result['wall_stats'], obstacle_n=len(result['obstacle_points'])))
    valid = [row for row in rows if row['valid']]
    return dict(capture=str(Path(capture).resolve()), active_scans=len(rows), valid_scans=len(valid),
                valid_fraction=len(valid)/max(1,len(rows)),
                reasons=dict(Counter(row['reason'] for row in rows)), rows=rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--recorded', required=True)
    parser.add_argument('--route', default=str(Path(__file__).parent/'global_path.json'))
    parser.add_argument('--output', default=str(Path(__file__).parent/'tunnel_wall_map.json'))
    parser.add_argument('--holdout')
    parser.add_argument('--validation-output')
    args = parser.parse_args()
    payload = build(args.recorded, args.route)
    if args.holdout:
        result = validate(TunnelMap(payload), args.holdout)
        payload['holdout_validation'] = {key:value for key,value in result.items() if key != 'rows'}
        if args.validation_output:
            Path(args.validation_output).write_text(json.dumps(result, indent=2))
    temporary = Path(args.output).with_suffix('.json.tmp')
    temporary.write_text(json.dumps(payload, indent=2)+'\n')
    temporary.replace(args.output)
    print(json.dumps({key:payload[key] for key in ('origin_xy','axis_heading_deg','left_offset_m',
        'right_offset_m','longitudinal_min_m','longitudinal_max_m','build','holdout_validation') if key in payload}))


if __name__ == '__main__':
    main()
