#!/usr/bin/env python3
"""Validate recorded roundabout driving evidence without ROS or control output."""
import argparse
import bisect
import collections
import hashlib
import json
import math
from pathlib import Path
import re


STOP_STATION_M = 885.1006875597407
FRONT_M, REAR_M, HALF_WIDTH_M = 3.845, .79, .946
CORE_SOURCES = ('gui.py', 'live_path_safety.py', 'traffic_planner.py',
                'junction_vehicle_motion.py', 'ring_entry.py', 'path_safety.py',
                'driving_plan.py', 'junction_launch.py')


def file_digest(path, algorithm='sha256'):
    digest = hashlib.new(algorithm)
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def file_version(path):
    try:
        value = path.stat()
        return value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns
    except OSError:
        return None


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError, OverflowError):
        return False


def json_file(path, issues, *, required=True):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError) as error:
        if required:
            issues.append(dict(code='missing_or_invalid_json', path=str(path), error=str(error)))
        return None


def json_lines(path, issues):
    records = []
    try:
        with path.open() as handle:
            for number, line in enumerate(handle, 1):
                if not line.strip():
                    continue
                try:
                    value = json.loads(line)
                    if not isinstance(value, dict):
                        raise ValueError('record is not an object')
                    records.append(value)
                except ValueError:
                    issues.append(dict(code='invalid_jsonl_record', path=str(path), line=number))
    except OSError as error:
        issues.append(dict(code='missing_jsonl', path=str(path), error=str(error)))
    return records


class RouteWindow:
    """Project the measured axle and body onto a recorded global-path window."""
    def __init__(self, route, stop, exit_station):
        values = route.get('poses_xyz_qxyzw', route.get('points', []))
        points = [(float(value[0]), float(value[1])) for value in values]
        if len(points) < 3 or not all(finite(v) for point in points for v in point):
            raise ValueError('global route has insufficient finite XY points')
        if route.get('frame_id', 'map') != 'map':
            raise ValueError('global route is not in map frame')
        station = 0.
        self.segments = []
        for a, b in zip(points, points[1:]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            length = math.hypot(dx, dy)
            if length > 1e-8 and station+length >= stop-80. and station <= exit_station+35.:
                self.segments.append((a[0], a[1], dx, dy, length, station))
            station += length
        if station < exit_station+2. or not self.segments:
            raise ValueError('global route does not cover the junction exit')
        vertices = [(s[0], s[1]) for s in self.segments]
        vertices += [(s[0]+s[2], s[1]+s[3]) for s in self.segments]
        self.bounds = (min(p[0] for p in vertices)-5., max(p[0] for p in vertices)+5.,
                       min(p[1] for p in vertices)-5., max(p[1] for p in vertices)+5.)
        self.length_m = station

    def nearby(self, x, y):
        a, b, c, d = self.bounds
        return a <= x <= b and c <= y <= d

    def project(self, x, y):
        best = None
        for ax, ay, dx, dy, length, station in self.segments:
            fraction = max(0., min(1., ((x-ax)*dx+(y-ay)*dy)/(length*length)))
            distance = math.hypot(x-ax-fraction*dx, y-ay-fraction*dy)
            if best is None or distance < best[1]:
                best = station+fraction*length, distance
        return best

    def body_station(self, ego, forward, lateral=0.):
        angle = math.radians(ego['heading_deg'])
        x = ego['x']+forward*math.cos(angle)-lateral*math.sin(angle)
        y = ego['y']+forward*math.sin(angle)+lateral*math.cos(angle)
        return self.project(x, y)[0]


def stopped_episodes(rows, threshold, max_gap):
    episodes, start, last = [], None, None
    for row in rows:
        stopped = row['speed_kmh'] <= threshold
        if start is not None and (not stopped or row['t']-last['t'] > max_gap):
            end = last['t'] if stopped else min(row['t'], last['t']+max_gap)
            episodes.append(dict(start=start['t'], end=end, wall_s=end-start['t'],
                                 xy=[start['x'], start['y']]))
            start = None
        if stopped:
            start = start or row
            last = row
    if start is not None:
        episodes.append(dict(start=start['t'], end=last['t'], wall_s=last['t']-start['t'],
                             xy=[start['x'], start['y']]))
    return episodes


def source_evidence(run_dir, issues, failures, source_root, compare_current):
    snapshot_path = run_dir/'roundabout_source_snapshot.json'
    manifest_path = run_dir/'versions.md5'
    snapshot = json_file(snapshot_path, issues, required=False)
    evidence = dict(current_sources_checked=bool(compare_current),
                    validates_current_checkout=False, validation_scope='recorded_run_only')
    comparisons = []
    if isinstance(snapshot, dict):
        values = snapshot.get('source_sha256')
        if (not isinstance(values, dict) or not all(name in values for name in CORE_SOURCES)
                or not all(re.fullmatch('[0-9a-fA-F]{64}', str(value)) for value in values.values())
                or not finite(snapshot.get('captured_wall'))
                or not re.fullmatch('[0-9a-fA-F]{64}', str(snapshot.get('scenario_sha256', '')))):
            issues.append(dict(code='invalid_source_snapshot'))
            return evidence
        evidence.update(kind='roundabout_source_snapshot', captured_wall=snapshot['captured_wall'],
            scenario_sha256=snapshot['scenario_sha256'], source_sha256=values,
            snapshot_sha256=file_digest(snapshot_path), complete_roundabout_runtime_snapshot=True)
        if compare_current:
            for name, expected in values.items():
                path = source_root/name
                actual = file_digest(path) if path.is_file() else None
                comparisons.append(dict(path=str(path), expected=expected, actual=actual,
                                        matches=actual == expected))
    elif manifest_path.exists():
        entries = []
        for line in manifest_path.read_text().splitlines():
            match = re.fullmatch(r'([0-9a-fA-F]{32})\s+\*?(.+)', line)
            if not match:
                issues.append(dict(code='invalid_versions_manifest', line=line))
                continue
            entries.append((Path(match.group(2)), match.group(1).lower()))
        evidence.update(kind='legacy_versions_md5', manifest_sha256=file_digest(manifest_path),
            recorded_files=[dict(path=str(path), md5=digest) for path, digest in entries],
            complete_roundabout_runtime_snapshot=False,
            unrecorded_core_sources=sorted(set(CORE_SOURCES)-{path.name for path, _ in entries}))
        if not entries:
            issues.append(dict(code='empty_source_manifest'))
        if compare_current:
            for path, expected in entries:
                actual = file_digest(path, 'md5') if path.is_file() else None
                comparisons.append(dict(path=str(path), expected=expected, actual=actual,
                                        matches=actual == expected))
            if evidence['unrecorded_core_sources']:
                issues.append(dict(code='incomplete_current_version_evidence',
                                   missing=evidence['unrecorded_core_sources']))
    else:
        issues.append(dict(code='missing_recorded_source_version'))
    evidence['comparisons'] = comparisons
    if any(not row['matches'] for row in comparisons):
        failures.append(dict(code='current_source_version_mismatch'))
    evidence['validates_current_checkout'] = bool(compare_current and comparisons
        and all(row['matches'] for row in comparisons)
        and evidence.get('complete_roundabout_runtime_snapshot'))
    return evidence


def evaluate_run(run_dir, raw_dir, *, stop_station_m=STOP_STATION_M,
                 exit_station_m=None, max_stop_s=15., require_full_lap=False,
                 compare_current_sources=False, source_root=None, max_sample_gap_s=.5):
    run_dir, raw_dir = Path(run_dir).resolve(), Path(raw_dir).resolve()
    source_root = Path(source_root or Path(__file__).resolve().parents[2]/'student_v3_claude')
    issues, failures = [], []
    run = json_file(run_dir/'run.json', issues) or {}
    session = json_file(raw_dir/'session.json', issues) or {}
    if not isinstance(run, dict) or not isinstance(session, dict):
        raise ValueError('run.json and session.json must contain objects')
    sid = run.get('session_id')
    if not isinstance(sid, str) or not sid or not isinstance(session.get('session_id'), str) or not session['session_id']:
        issues.append(dict(code='missing_session_identity'))
    if sid and session.get('session_id') and sid != session['session_id']:
        failures.append(dict(code='run_raw_session_mismatch', run=sid, raw=session['session_id']))
    gui_path = run_dir/('student_gui_'+str(sid)+'.jsonl')
    if not gui_path.exists():
        candidates = list(run_dir.glob('student_gui_*.jsonl'))
        if len(candidates) == 1:
            gui_path = candidates[0]
    artifact_paths = [run_dir/'run.json', raw_dir/'session.json', gui_path,
                      raw_dir/'ego_raw.jsonl', raw_dir/'route.json', raw_dir/'collisions.json',
                      run_dir/'lap_complete.json', run_dir/'roundabout_source_snapshot.json', run_dir/'versions.md5']
    before = {path: file_version(path) for path in artifact_paths}
    gui = json_lines(gui_path, issues)
    gui = [row for row in gui if finite(row.get('t'))]
    gui.sort(key=lambda row: row['t'])
    if not gui:
        issues.append(dict(code='no_gui_evidence'))
    gui_start, gui_end = (gui[0]['t'], gui[-1]['t']) if gui else (None, None)
    complete = (finite(run.get('ended_wall_ns')) and float(run['ended_wall_ns']) > 0.
                and type(run.get('exit_code')) is int and run['exit_code'] == 0)
    if not complete:
        issues.append(dict(code='run_incomplete', exit_code=run.get('exit_code')))
    collision = json_file(raw_dir/'collisions.json', issues)
    if (isinstance(collision, dict) and isinstance(collision.get('events'), list)
            and finite(collision.get('messages')) and float(collision['messages']) >= 0.):
        if collision['events'] or float(collision['messages']) > 0.:
            failures.append(dict(code='collision', event_count=len(collision['events']),
                                 messages=collision['messages']))
    elif collision is not None:
        issues.append(dict(code='invalid_collision_evidence'))
    if exit_station_m is None:
        regions = (run.get('intersection_perception') or {}).get('regions', [])
        regions += [(row.get('intersection_lidar') or {}).get('active_region') or {} for row in gui]
        exits = [float(region['exit_station_m']) for region in regions
                 if finite(region.get('stop_station_m')) and finite(region.get('exit_station_m'))
                 and abs(float(region['stop_station_m'])-stop_station_m) < .5
                 and float(region['exit_station_m']) > stop_station_m+5.]
        exit_station_m = max(exits) if exits else stop_station_m+30.
        exit_origin = 'recorded_active_region' if exits else 'fixed_map_30m_exit_window'
    else:
        exit_origin = 'explicit_argument'
    if not all(finite(v) for v in (stop_station_m, exit_station_m, max_stop_s, max_sample_gap_s)) or not (
            exit_station_m > stop_station_m and max_stop_s > 0. and max_sample_gap_s > 0.):
        raise ValueError('station bounds and duration thresholds must be finite and positive')
    route_value = json_file(raw_dir/'route.json', issues)
    route = None
    if isinstance(route_value, dict):
        try:
            route = RouteWindow(route_value, stop_station_m, exit_station_m)
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            issues.append(dict(code='invalid_global_route', error=str(error)))
    elif route_value is not None:
        issues.append(dict(code='invalid_global_route', error='route JSON must contain an object'))
    egos = []
    previous = None
    for row in json_lines(raw_dir/'ego_raw.jsonl', issues):
        if row.get('type') != 'ego':
            continue
        if not all(finite(row.get(key)) for key in ('t_us', 'x', 'y', 'heading_deg', 'vx', 'vy')):
            issues.append(dict(code='invalid_ego_measurement'))
            continue
        t = float(row['t_us'])/1e6
        if previous is not None and t < previous-1e-6:
            issues.append(dict(code='ego_clock_rewind', previous=previous, current=t))
        previous = t
        if gui_start is not None and gui_start <= t <= gui_end:
            egos.append(dict(t=t, x=float(row['x']), y=float(row['y']),
                heading_deg=float(row['heading_deg']), speed_kmh=math.hypot(float(row['vx']), float(row['vy']))*3.6))
    egos.sort(key=lambda row: row['t'])
    if len(egos) < 2:
        issues.append(dict(code='insufficient_actual_ego_evidence'))
    approaching = False
    front_entry, rear_exit = None, None
    junction_rows = []
    if route:
        for ego in egos:
            if not route.nearby(ego['x'], ego['y']):
                continue
            station, distance = route.project(ego['x'], ego['y'])
            if not stop_station_m-40. <= station <= exit_station_m+6. or distance > 3.:
                continue
            if rear_exit is not None:
                continue
            junction_rows.append(dict(ego, station_m=station, route_distance_m=distance))
            if front_entry is None:
                front_station = route.body_station(ego, FRONT_M)
                approaching |= front_station < stop_station_m-.5
                if front_station >= stop_station_m:
                    front_entry = dict(ego, front_station_m=front_station)
            if front_entry is not None and station >= exit_station_m-5.:
                corner_stations = [route.body_station(ego, x, y)
                                   for x in (-REAR_M, FRONT_M) for y in (-HALF_WIDTH_M, HALF_WIDTH_M)]
                if min(corner_stations) >= exit_station_m:
                    rear_exit = dict(ego, minimum_body_station_m=min(corner_stations))
        if complete or junction_rows:
            if front_entry is None:
                failures.append(dict(code='roundabout_not_entered'))
            elif not approaching:
                issues.append(dict(code='entry_started_inside_or_approach_unobserved'))
            if rear_exit is None:
                failures.append(dict(code='roundabout_body_did_not_clear_exit'))
    gaps = [b['t']-a['t'] for a, b in zip(junction_rows, junction_rows[1:])]
    if gaps and max(gaps) > max_sample_gap_s:
        issues.append(dict(code='junction_ego_sampling_gap', max_wall_s=max(gaps)))
    full_stops = stopped_episodes(junction_rows, .1, max_sample_gap_s)
    standstill = stopped_episodes(junction_rows, .5, max_sample_gap_s)
    longest = max((row['wall_s'] for row in standstill), default=0.)
    if longest >= max_stop_s:
        failures.append(dict(code='long_roundabout_standstill', wall_s=longest, limit_wall_s=max_stop_s))
    overrides = []
    for row in gui:
        traffic, yielded = row.get('traffic_plan') or {}, row.get('yield') or {}
        if (row.get('stop_line') is None and traffic.get('state') == 'CROSSING_CAUTION'
                and finite(traffic.get('target_mps')) and finite(yielded.get('v_kmh'))):
            gap = float(yielded['v_kmh'])-float(traffic['target_mps'])*3.6
            if gap > .2+1e-9:
                overrides.append(dict(t=row['t'], traffic_target_mps=traffic['target_mps'],
                                      yield_kmh=yielded['v_kmh'], excess_kmh=gap))
    if overrides:
        failures.append(dict(code='following_only_contact_cap_override', count=len(overrides), examples=overrides[:20]))
    lap = json_file(run_dir/'lap_complete.json', issues, required=False)
    sector_rows = dict(highway=[], tunnel=[])
    stale_sector_samples = collections.Counter()
    times = [row['t'] for row in gui]
    for ego in egos:
        index = bisect.bisect_right(times, ego['t'])-1
        if index < 0:
            continue
        mode = str(gui[index].get('mode', ''))
        sector = 'highway' if mode == 'HIGHWAY' else 'tunnel' if mode.startswith('TUNNEL') else None
        if sector:
            sector_rows[sector].append(ego)
            if ego['t']-times[index] > .6:
                stale_sector_samples[sector] += 1
    sectors = {}
    for name, values in sector_rows.items():
        sectors[name] = dict(actual_samples=len(values),
            span_wall_s=values[-1]['t']-values[0]['t'] if values else 0.,
            min_actual_kmh=min((row['speed_kmh'] for row in values), default=None),
            stopped_samples=sum(row['speed_kmh'] <= .5 for row in values))
        if require_full_lap:
            if stale_sector_samples[name]:
                issues.append(dict(code='sector_mode_evidence_gap', sector=name,
                                   actual_samples=stale_sector_samples[name]))
            if len(values) < 2 or sectors[name]['span_wall_s'] < .5:
                failures.append(dict(code='missing_actual_sector_coverage', sector=name))
            if sectors[name]['stopped_samples']:
                failures.append(dict(code='actual_sector_stop', sector=name,
                                     count=sectors[name]['stopped_samples']))
    if require_full_lap and not (isinstance(lap, dict) and lap.get('lap_complete') is True):
        failures.append(dict(code='full_lap_marker_missing'))
    provenance = source_evidence(run_dir, issues, failures, source_root, compare_current_sources)
    first_motion = next((row['t'] for row in egos if row['speed_kmh'] > .5), None)
    if (first_motion is not None and finite(provenance.get('captured_wall'))
            and provenance['captured_wall'] > first_motion+.05):
        issues.append(dict(code='source_snapshot_captured_after_actual_motion',
                           captured_wall=provenance['captured_wall'], first_motion=first_motion))
    artifacts = {}
    for path, version in before.items():
        if version is not None:
            artifacts[str(path)] = dict(sha256=file_digest(path), bytes=path.stat().st_size)
        if file_version(path) != version:
            issues.append(dict(code='artifact_changed_during_validation', path=str(path)))
    verdict = 'FAIL' if failures else 'INCOMPLETE' if issues else 'PASS'
    return dict(schema=1, verdict=verdict, run_dir=str(run_dir), raw_dir=str(raw_dir), session_id=sid,
        failures=failures, incomplete_evidence=issues,
        policy=dict(require_full_lap=bool(require_full_lap), max_stop_wall_s=max_stop_s,
            stop_station_m=stop_station_m, exit_station_m=exit_station_m, exit_station_origin=exit_origin,
            ego_body_m=dict(front=FRONT_M, rear=REAR_M, width=HALF_WIDTH_M*2.),
            recorded_stop_line_present=any(row.get('stop_line') is not None for row in gui),
            actual_velocity='hypot(ego_raw.vx, ego_raw.vy)*3.6; no RTF scaling',
            duration='wall seconds from raw t_us, cropped to recorded GUI timestamps'),
        roundabout=dict(approach_observed=approaching, front_entry=front_entry, whole_body_exit=rear_exit,
            actual_samples=len(junction_rows), full_stop_bursts=full_stops,
            full_stop_count_ge_0_1s=sum(row['wall_s'] >= .1 for row in full_stops),
            full_stop_wall_s=sum(row['wall_s'] for row in full_stops), longest_standstill_wall_s=longest),
        driving_window=dict(first_gui=gui_start, last_gui=gui_end,
            actual_samples=len(egos), cleanup_samples_excluded=previous is not None and gui_end is not None and previous > gui_end),
        contact_cap_overrides=overrides, sectors=sectors, collisions=collision, lap_complete=lap,
        source_evidence=provenance, artifacts=artifacts)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True, type=Path, help='Exact recorded follow run directory containing run.json')
    parser.add_argument('--raw-dir', required=True, type=Path, help='Matching raw session directory containing ego_raw.jsonl and route.json')
    parser.add_argument('--output', type=Path, help='Optional JSON file; stdout always receives JSON')
    parser.add_argument('--stop-station-m', type=float, default=STOP_STATION_M)
    parser.add_argument('--exit-station-m', type=float)
    parser.add_argument('--max-stop-s', type=float, default=15.)
    parser.add_argument('--require-full-lap', action='store_true')
    parser.add_argument('--compare-current-sources', action='store_true', help='Fail on a recorded/current source hash mismatch')
    parser.add_argument('--source-root', type=Path, help='Directory of the runtime Python sources for SHA256 snapshot comparison')
    args = parser.parse_args(argv)
    try:
        result = evaluate_run(args.run_dir, args.raw_dir, stop_station_m=args.stop_station_m,
            exit_station_m=args.exit_station_m, max_stop_s=args.max_stop_s,
            require_full_lap=args.require_full_lap,
            compare_current_sources=args.compare_current_sources, source_root=args.source_root)
    except (OSError, TypeError, ValueError, OverflowError) as error:
        result = dict(schema=1, verdict='INCOMPLETE', failures=[],
                      incomplete_evidence=[dict(code='validation_input_error', error=str(error))])
    output = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    if args.output:
        try:
            args.output.write_text(output)
        except OSError as error:
            result['incomplete_evidence'].append(dict(code='json_output_write_failed', error=str(error)))
            if result['verdict'] == 'PASS':
                result['verdict'] = 'INCOMPLETE'
            output = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    print(output, end='')
    return 0 if result['verdict'] == 'PASS' else 1 if result['verdict'] == 'FAIL' else 2


if __name__ == '__main__':
    raise SystemExit(main())
