#!/usr/bin/env python3
"""Scenario-side NPC activation for generated K-City competition scenes.

Uses follower progress only to release simulator actors. Actor truth is never
published to perception or used to choose the Ego's path or control commands.
"""
import json
import re
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import auto_collect as ac

STATUS = ac.DATA / 'autodrive/scenarios/competition_traffic_status.json'
ZONES = {'roundabout': (1740, ['20000', '20001', '20002']),
         'highway': (2150, ['20003', '20004', '20005']),
         'tunnel': (3740, ['20006', '20007'])}
ADDED = {'roundabout': ['20008'], 'highway': ['20009', '20010', '20011', '20012', '20013'], 'tunnel': []}
LOAD = re.compile(r'The scenario file has been Load in \[([^\]]+\.json)\] Complete')
running = True


def set_pause(ids, enable):
    if not ids:
        return
    ac.rpc("from runtime.morai_rpc import MoraiRPC\nr=MoraiRPC()\n"
           "for uid in " + repr(ids) + ":\n"
           " r.call('actor.Actor','SetPause',{'actor_info':{'id':{'value':uid},"
           "'object_type':'OBJECT_TYPE_VEHICLE'},'enable':" + repr(enable) + "},timeout=3)\n")


def recognized(scene):
    """Recognize current and archived generator scenes by their native starts."""
    vehicles = {str(v['UNIQUEID']): v for v in scene.get('vehicleList', [])}
    expected = {uid for _, ids in ZONES.values() for uid in ids}
    expanded = expected | {uid for ids in ADDED.values() for uid in ids}
    previous = expected | {'20008', '20009', '20010'}
    if set(vehicles) not in (expected, previous, expanded) or scene.get('spawnPointList'):
        return False
    starts = {'20000': 'A2256W000133', '20001': 'A2256W000134', '20002': 'A2256W000135',
              '20003': 'A2256W000411', '20004': 'A2256W000409', '20005': 'A2256W000420',
              '20006': 'A2256W000126', '20007': 'A2256W000126',
              '20008': 'A2256W000136', '20009': 'A2256W000411', '20010': 'A2256W000420',
              '20011': 'A2256W000410', '20012': 'A2256W000431', '20013': 'A2256W000435'}
    return all(vehicles[uid]['initPosition']['initLink'] == starts[uid] for uid in vehicles)


def save(state):
    STATUS.parent.mkdir(parents=True, exist_ok=True)
    tmp = STATUS.with_suffix('.tmp')
    tmp.write_text(json.dumps(dict(state, updated_at=time.time()), ensure_ascii=False, indent=2)+'\n')
    tmp.replace(STATUS)


def stop(*_):
    global running
    running = False


def main():
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    state = dict(scenario=None, ready=False, released=[], current_idx=None, error=None)
    offset = 0
    zones = ZONES
    while running:
        try:
            if not ac.PLAYER_LOG.exists():
                time.sleep(1)
                continue
            size = ac.PLAYER_LOG.stat().st_size
            if size < offset:
                offset = 0
            with ac.PLAYER_LOG.open('rb') as stream:
                stream.seek(offset)
                lines = stream.read().decode(errors='replace')
                offset = stream.tell()
            loads = list(LOAD.finditer(lines))
            if loads:
                name = loads[-1].group(1)
                scene = json.loads((ac.SCENARIO_DIR / name).read_text())
                state = dict(scenario=name, ready=False, released=[], current_idx=None, error=None, load_token=offset)
                if recognized(scene):
                    actual_ids = {str(v['UNIQUEID']) for v in scene['vehicleList']}
                    zones = {zone: (trigger, ids+[uid for uid in ADDED[zone] if uid in actual_ids])
                             for zone, (trigger, ids) in ZONES.items()}
                    set_pause([uid for _, ids in zones.values() for uid in ids], True)
                    state['counts'] = {zone: len(ids) for zone, (_, ids) in zones.items()}
                    state['ready'] = True
                    print(json.dumps(dict(event='npc_paused', scenario=name)), flush=True)
                save(state)
            if state['ready']:
                status = ac.http(ac.WEB + '/api/status', timeout=8)
                ros = status.get('ros', {})
                if status.get('job', {}).get('running') and ros.get('follower_running'):
                    idx = (ros.get('path') or {}).get('nearest_idx')
                    state['current_idx'] = idx
                    if idx is not None:
                        for zone, (trigger, ids) in zones.items():
                            if zone not in state['released'] and idx >= trigger:
                                set_pause(ids, False)
                                state['released'].append(zone)
                                print(json.dumps(dict(event='npc_released', zone=zone, route_idx=idx)), flush=True)
                state['error'] = None
                save(state)
        except Exception as error:
            state['error'] = str(error)
            save(state)
            print(json.dumps(dict(event='traffic_controller_error', error=str(error))), flush=True)
            time.sleep(2)
        time.sleep(.3)


if __name__ == '__main__':
    main()
