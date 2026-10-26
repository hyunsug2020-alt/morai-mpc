#!/usr/bin/env python3
"""Generate, load and test successive competition scenarios on the single PC.

Uses the existing follower without starting camera dataset collection. Keeps
scenario configurations, run paths and collision verdicts locally for replay.
"""
import argparse
import fcntl
import json
import random
import re
import shutil
import signal
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
import auto_collect as ac

STATE = ac.DATA / 'autodrive/competition_cycle'
TRAFFIC_STATUS = ac.DATA / 'autodrive/scenarios/competition_traffic_status.json'


class LapProgress:
    def __init__(self, length, now):
        self.length = length
        self.mid = self.end = self.moving = False
        self.previous_idx = None
        self.previous_t = self.last_move = now
        self.max_idx = 0

    def observe(self, idx, now):
        self.max_idx = max(self.max_idx, idx)
        if self.length * .3 < idx < self.length * .6:
            self.mid = True
        if self.mid and idx > self.length * .95:
            self.end = True
        if self.previous_idx is not None:
            step = (idx - self.previous_idx) % self.length
            if 2 < step <= max(150, 60 * (now - self.previous_t)):
                self.last_move, self.moving = now, True
        self.previous_idx, self.previous_t = idx, now
        return self.mid and self.end and idx < 100


def scenario_config(rng, index):
    return dict(seed='contest-cycle-{}-{:04d}-{:08x}'.format(time.strftime('%Y%m%d-%H%M%S'), index, rng.getrandbits(32)),
                template='morai_hard_13' if index == 1 else rng.choice(ac.TEMPLATES),
                preset='missions', events=14, start=204, end=1990, corridor=3.5, gap=3,
                egoClearance=25, passage=2.6, pedestrianSpeed=rng.choice([3, 3.5, 4]), activeDistance=18,
                keepTraffic=False, keepObjects=False, keepPedestrians=False,
                kinds=['scatter', 'parked', 'crossing'], placementVersion=2, lanePolicyVersion=2,
                missionPlan='competition-signals-tunnel-v2', missionDistribution='spread',
                missionCounts=dict(static=8 if index == 1 else rng.randint(7, 9), dynamic=rng.randint(2, 3)),
                missionTraffic=True, missionTrafficCounts=dict(roundabout=4, highway=8, tunnel=2))


def atomic_json(path, value):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    tmp.replace(path)


class Cycle:
    def __init__(self, args):
        self.args = args
        self.stopping = False
        self.owned_start = None
        self.current = dict(phase='starting', round=0, scenario=None)
        self.route_n = ac.route_length()

    def status(self, **values):
        self.current.update(values, updated_at=time.time())
        atomic_json(STATE / 'status.json', self.current)

    def log(self, event, **values):
        print(json.dumps(dict(event=event, time=time.time(), **values), ensure_ascii=False), flush=True)

    def stop_signal(self, *_):
        self.stopping = True

    def stop_owned(self):
        state = ac.job()
        if state.get('running') and self.owned_start is not None and state.get('started_at') == self.owned_start:
            ac.http(ac.WEB+'/api/stop', {}, timeout=30)
            if not ac.wait_job_idle(60):
                raise RuntimeError('소유한 주행 작업의 종료 확인 실패')
        self.owned_start = None

    def generate(self, index, rng):
        config = scenario_config(rng, index)
        result = ac.http(ac.STUDIO+'/api/save-to-morai', dict(config=config, count=1,
                        cycleTemplates=False, diversify=False), timeout=60)
        name = result['saved'][0]
        if not name.endswith('.json'):
            name += '.json'
        shutil.copyfile(ac.SINGLE_PC_NETWORK, ac.SCENARIO_DIR/'EgoNetwork'/(Path(name).stem+'_MN.json'))
        scene = json.loads((ac.SCENARIO_DIR/name).read_text())
        npc_count = sum(config['missionTrafficCounts'].values())
        if len(scene['vehicleList']) != npc_count or scene['spawnPointList']:
            raise RuntimeError('구간 NPC {}대 또는 차량 스폰 제거 검증 실패'.format(npc_count))
        if len(scene['objectList']) != config['missionCounts']['static'] or len(scene['pedestrianList']) != config['missionCounts']['dynamic']:
            raise RuntimeError('요청한 장애물·보행자 수 검증 실패')
        atomic_json(STATE / ('round_{:04d}_scenario.json'.format(index)), dict(name=name, config=config, scene=scene))
        return name, config, scene

    def load(self, name, scene):
        ac.load_scenario(name)
        deadline = time.monotonic()+20
        while time.monotonic() < deadline and not self.stopping:
            try:
                state = json.loads(TRAFFIC_STATUS.read_text())
            except (OSError, ValueError):
                state = {}
            if state.get('scenario') == name and state.get('ready') and not state.get('released') and not state.get('error'):
                break
            time.sleep(.3)
        else:
            if self.stopping:
                return
            raise RuntimeError('NPC 구간별 대기 제어 준비 실패')
        actual = json.loads(ac.rpc("from runtime.morai_rpc import MoraiRPC\nimport json\nr=MoraiRPC()\n"
                                  "print(json.dumps(r.call('actor.Actor','GetAllActorsState',{'vehicle':True,'pedestrian':True,'obstacle':True},timeout=4)['states']))"))
        expected = {'Ego'} | {str(a['UNIQUEID']) for key in ['vehicleList', 'pedestrianList', 'objectList'] for a in scene[key]}
        ids = {a['actor_info']['id']['value'] for a in actual}
        if ids != expected:
            raise RuntimeError('실제 로드 객체 불일치: 누락 {} / 추가 {}'.format(sorted(expected-ids), sorted(ids-expected)))
        deadline = time.monotonic()+10
        capture = STATE/'round_{:04d}_data'.format(self.current['round'])/'camera_capture.json'
        while time.monotonic() < deadline and not self.stopping:
            try:
                recording = json.loads(capture.read_text())
            except (OSError, ValueError):
                recording = {}
            if recording.get('state') == 'recording':
                return
            time.sleep(.2)
        if not self.stopping:
            raise RuntimeError('충돌 전후 영상 기록기 준비 실패')

    def drive(self):
        if ac.job().get('running'):
            raise RuntimeError('다른 주행이 실행 중임')
        ac.ensure_network()
        ac.http(ac.WEB+'/api/start', dict(action='autodrive_follow', options=dict(minutes=self.args.max_minutes,
                                                                             network_profile='single_pc', drive_mode='e2e')), timeout=30)
        job = ac.job()
        self.owned_start = job.get('started_at')
        self.status(phase='starting_drive')
        started = time.monotonic()
        lap = LapProgress(self.route_n, started)
        run = None
        checkpoints = []
        outcome = 'time_limit'
        while not self.stopping and time.monotonic()-started < self.args.max_minutes*60+90:
            status = ac.http(ac.WEB+'/api/status', timeout=15)
            job = status.get('job', {})
            if job.get('started_at') != self.owned_start:
                self.stopping = True
                outcome = 'control_changed'
                break
            for line in job.get('logs', []):
                match = re.search(r'run=(follow-[\w-]+)', line)
                if match:
                    run = ac.DATA / 'autodrive/follow_runs' / match.group(1)
            if not job.get('running'):
                if job.get('exit_code') == 130:
                    self.stopping, outcome = True, 'stopped_by_user'
                else:
                    outcome = 'time_limit' if job.get('exit_code') in (0, None) else ('runtime_error' if lap.moving else 'startup_error')
                break
            ros = status.get('ros', {})
            if ros.get('follower_running'):
                idx = (ros.get('path') or {}).get('nearest_idx')
                if idx is not None:
                    row = dict(time=time.time(), route_idx=idx, ego=ros.get('ego'), ctrl_hz=ros.get('ctrl_hz'))
                    checkpoints.append(row)
                    if lap.observe(int(idx), time.monotonic()):
                        outcome = 'lap_complete'
                        break
                    if lap.moving and time.monotonic()-lap.last_move > self.args.stuck_seconds:
                        outcome = 'stuck'
                        break
                    self.status(phase='driving', route_idx=idx, progress_pct=round(idx/self.route_n*100, 1),
                                ego=ros.get('ego'), run_path=str(run) if run else None)
            else:
                # Model startup can exceed 30 seconds. Keep the evidence recorder
                # attached while initialization waits for the follower.
                self.status(phase='starting_drive', run_path=str(run) if run else None)
            time.sleep(2)
        if self.stopping and outcome not in ('control_changed', 'stopped_by_user'):
            outcome = 'stopped_by_user'
        self.stop_owned()
        return dict(outcome=outcome, run_path=str(run) if run else None,
                    duration_s=round(time.monotonic()-started, 1), max_route_idx=lap.max_idx, checkpoints=checkpoints)

    def audit(self, result):
        if not result.get('run_path'):
            return
        import rosbag
        samples, collisions = 0, []
        run = Path(result['run_path'])
        for path in run.glob('*.bag'):
            with rosbag.Bag(str(path)) as bag:
                for _, msg, stamp in bag.read_messages(topics=['/CollisionData']):
                    samples += 1
                    if msg.collision_object:
                        collisions.append(dict(time=stamp.to_sec(), objects=str(msg.collision_object)))
        result.update(collision_samples=samples, collision_events=len(collisions), collisions=collisions)
        traffic = json.loads(TRAFFIC_STATUS.read_text())
        result['activated_regions'] = traffic.get('released', [])

    def main(self):
        signal.signal(signal.SIGTERM, self.stop_signal)
        signal.signal(signal.SIGINT, self.stop_signal)
        rng = random.Random()
        existing = list(STATE.glob('round_*_*.json'))
        index = max([int(p.name.split('_')[1]) for p in existing] or [0])
        finished = 0
        try:
            while not self.stopping and (not self.args.laps or finished < self.args.laps):
                free_gb = shutil.disk_usage(ac.DATA / 'autodrive').free/1e9
                if free_gb < self.args.min_free_gb:
                    self.status(phase='waiting_disk', free_gb=round(free_gb, 2))
                    time.sleep(5)
                    continue
                if ac.job().get('running'):
                    self.status(phase='waiting_drive')
                    time.sleep(2)
                    continue
                index += 1
                name = None
                self.status(phase='generating', round=index, route_idx=None, scenario=None, last_error=None,
                            ego=None, progress_pct=0, config=None)
                try:
                    name, config, scene = self.generate(index, rng)
                    if self.stopping:
                        break
                    self.status(phase='loading', scenario=name, config=config)
                    self.load(name, scene)
                    if self.stopping:
                        break
                    self.log('drive_start', round=index, scenario=name, counts=config['missionCounts'],
                             npcs=sum(config['missionTrafficCounts'].values()))
                    result = self.drive()
                    self.status(phase='checking', run_path=result.get('run_path'),
                                progress_pct=100 if result['outcome'] == 'lap_complete' else self.current.get('progress_pct', 0))
                    try:
                        self.audit(result)
                    except Exception as error:
                        result['audit_error'] = str(error)
                    result.update(round=index, scenario=name, config=config)
                except Exception as error:
                    self.stop_owned()
                    result = dict(round=index, scenario=name, outcome='error', error=str(error))
                atomic_json(STATE/('round_{:04d}_result.json'.format(index)), result)
                summary = {k:v for k,v in result.items() if k not in ('checkpoints', 'collisions')}
                with (STATE/'results.jsonl').open('a') as stream:
                    stream.write(json.dumps(summary, ensure_ascii=False)+'\n')
                finished += 1
                self.log('drive_finished', round=index, scenario=name, outcome=result['outcome'], collisions=result.get('collision_events'))
                self.status(phase='between_runs', completed=finished, last_result=summary)
                if not self.stopping:
                    time.sleep(3 if result['outcome'] != 'error' else 10)
        finally:
            self.stop_owned()
            self.status(phase='stopped')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--laps', type=int, default=0, help='0 repeats until stopped')
    parser.add_argument('--max-minutes', type=float, default=10)
    parser.add_argument('--stuck-seconds', type=float, default=90)
    parser.add_argument('--min-free-gb', type=float, default=5)
    args = parser.parse_args()
    if args.laps < 0 or args.max_minutes <= 0 or args.stuck_seconds <= 0 or args.min_free_gb <= 0:
        parser.error('양의 제한값 또는 --laps 0 필요함')
    STATE.mkdir(parents=True, exist_ok=True)
    with (STATE/'cycle.lock').open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('시나리오 반복 주행이 이미 실행 중임')
        Cycle(args).main()


if __name__ == '__main__':
    main()
