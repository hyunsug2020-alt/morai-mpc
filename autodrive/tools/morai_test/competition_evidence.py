#!/usr/bin/env python3
"""Local collision evidence for Scenario Studio runs; never publishes control data.

The follower already records full-run sensor, localization, perception and control
bags. This process keeps the existing shared-memory camera/semantic feed in a
bounded RAM window and persists it only around contact. Finished runs receive a
manifest, structured collision context and small, independently readable bags.
"""
import collections
import concurrent.futures
import hashlib
import json
import math
import queue
import shutil
import signal
import threading
import time
import zipfile
from pathlib import Path

import competition_cycle as cc

PRE_SECONDS = 10
POST_SECONDS = 5
POLL_HZ = 15
MAX_BUFFER_BYTES = 192 * 1024 * 1024
FEED = Path('/dev/shm/alpamayo_semantic')
FEED_FILES = ('frames.npz', 'masks.npz', 'decision.json', 'fusion.json', 'gui_cams.jpg', 'gui_plan.jpg')
CONTEXT_TOPICS = ['/localization/ego_status', '/Competition_topic', '/ctrl_cmd',
                  '/avoid_target_vel', '/lane_change_mode', '/student/path_status',
                  '/perception/objects', '/perception/lidar_vehicles', '/mpc_status']
REQUIRED_TOPICS = ['/CollisionData', '/lidar/points', '/localization/ego_status',
                   '/ctrl_cmd', '/avoid_waypoints', '/perception/objects']


def read_json(path, default=None):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return default


def message_dict(value):
    if hasattr(value, '__slots__'):
        return {key: message_dict(getattr(value, key)) for key in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [message_dict(v) for v in value]
    return value


def evidence_dir(index):
    return cc.STATE / 'round_{:04d}_data'.format(index)


def collision_episodes(records, gap=1.0):
    """Duplicate/continuous contact messages are one episode, with originals kept."""
    episodes = []
    for record in sorted(records, key=lambda r: r['time']):
        if not episodes or record['time'] - episodes[-1]['last_time'] > gap:
            episodes.append(dict(first_time=record['time'], last_time=record['time'], messages=[], object_ids=[]))
        event = episodes[-1]
        event['last_time'] = record['time']
        event['messages'].append(record)
        event['object_ids'] = sorted(set(event['object_ids']) | {str(obj.get('unique_id')) for obj in record['objects']})
    return episodes


class CameraWindow:
    """Nonblocking contact admission; a separate writer owns the disk files."""
    def __init__(self, index):
        self.root = evidence_dir(index)
        self.root.mkdir(parents=True, exist_ok=True)
        self.started_ns = time.time_ns()
        self.buffer = collections.deque()
        self.buffer_bytes = 0
        self.memory_trimmed = 0
        self.tokens = {}
        self.events = []
        self.contacts = queue.Queue(maxsize=1024)
        self.writes = queue.Queue(maxsize=256)
        self.dropped_contacts = self.dropped_samples = self.write_errors = 0
        self.saved = collections.Counter()
        self.seen = collections.Counter()
        self.closed = False
        self.writer = threading.Thread(target=self._writer, daemon=True, name='collision-camera-writer')
        self.writer.start()
        self.last_status = 0

    def contact(self, objects, wall, ros_time):
        try:
            self.contacts.put_nowait(dict(time=wall, ros_time=ros_time, objects=objects))
        except queue.Full:
            self.dropped_contacts += 1

    def enqueue(self, event, sample):
        try:
            self.writes.put_nowait((event['id'], sample))
        except queue.Full:
            self.dropped_samples += 1

    def sample(self, now=None):
        now = time.time() if now is None else now
        while True:
            try:
                contact = self.contacts.get_nowait()
            except queue.Empty:
                break
            if not self.events or contact['time'] - self.events[-1]['last_time'] > 1:
                event = dict(id=len(self.events)+1, first_time=contact['time'], last_time=contact['time'],
                             messages=[], pre_buffer_start=self.buffer[0]['time'] if self.buffer else None)
                self.events.append(event)
                for sample in self.buffer:
                    if contact['time']-PRE_SECONDS <= sample['time'] <= contact['time']:
                        self.enqueue(event, sample)
            event = self.events[-1]
            event['last_time'] = contact['time']
            event['messages'].append(contact)
        files = {}
        for name in FEED_FILES:
            try:
                # Read one inode: atomic replacement cannot mix bytes and timestamps.
                with (FEED/name).open('rb') as stream:
                    import os
                    stat = os.fstat(stream.fileno())
                    token = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
                    if token == self.tokens.get(name) or stat.st_mtime_ns < self.started_ns or now-stat.st_mtime > 1:
                        continue
                    if stat.st_size > 8 * 1024 * 1024:
                        continue
                    data = stream.read()
                self.tokens[name] = token
                files[name] = dict(data=data, source_mtime_ns=stat.st_mtime_ns)
                self.seen[name] += 1
            except OSError:
                continue
        if files:
            sample = dict(time=now, wall_ns=time.time_ns(), files=files, size=sum(len(f['data']) for f in files.values()))
            self.buffer.append(sample)
            self.buffer_bytes += sample['size']
            for event in self.events:
                if event['first_time']-PRE_SECONDS <= now <= event['last_time']+POST_SECONDS:
                    self.enqueue(event, sample)
        while self.buffer and (now-self.buffer[0]['time'] > PRE_SECONDS or self.buffer_bytes > MAX_BUFFER_BYTES):
            if self.buffer_bytes > MAX_BUFFER_BYTES:
                self.memory_trimmed += 1
            self.buffer_bytes -= self.buffer.popleft()['size']
        if now-self.last_status >= 1:
            self.status('recording')
            self.last_status = now

    def _writer(self):
        while True:
            item = self.writes.get()
            if item is None:
                return
            index, sample = item
            try:
                target = self.root/'incident_{:04d}'.format(index)/'samples'/str(sample['wall_ns'])
                target.mkdir(parents=True, exist_ok=True)
                saved = []
                for name, value in sample['files'].items():
                    (target/name).write_bytes(value['data'])
                    self.saved[name] += 1
                    saved.append(dict(name=name, path=str((target/name).relative_to(self.root)),
                                      bytes=len(value['data']), source_mtime_ns=value['source_mtime_ns']))
                with (target.parent.parent/'index.jsonl').open('a') as stream:
                    stream.write(json.dumps(dict(time=sample['time'], received_wall_ns=sample['wall_ns'], files=saved))+'\n')
            except OSError:
                self.write_errors += 1

    def status(self, phase):
        cc.atomic_json(self.root/'camera_capture.json', dict(state=phase, started_wall_ns=self.started_ns,
                       ended_wall_ns=time.time_ns() if phase != 'recording' else None,
                       pre_seconds=PRE_SECONDS, post_seconds=POST_SECONDS, poll_hz=POLL_HZ,
                       source='existing inference shared-memory feed; original JPEG/NPZ bytes',
                       buffer_limit_bytes=MAX_BUFFER_BYTES, memory_trimmed_samples=self.memory_trimmed,
                       dropped_contacts=self.dropped_contacts, dropped_samples=self.dropped_samples,
                       write_errors=self.write_errors, seen_files=dict(self.seen), saved_files=dict(self.saved),
                       incidents=self.events))

    def close(self):
        self.closed = True
        # Drain pending contact admission before flushing; no new frames are needed.
        self.sample()
        self.writes.put(None)
        self.writer.join()
        self.status('complete' if not (self.write_errors or self.dropped_samples or self.dropped_contacts) else 'incomplete')
        self.buffer.clear()


def snapshot_scenario(index, live=False):
    root = evidence_dir(index)
    root.mkdir(parents=True, exist_ok=True)
    archive = read_json(cc.STATE/'round_{:04d}_scenario.json'.format(index), {})
    if archive:
        cc.atomic_json(root/'scenario_config.json', {k:v for k,v in archive.items() if k != 'scene'})
        cc.atomic_json(root/'scenario.json', archive['scene'])
        network = cc.ac.SCENARIO_DIR/'EgoNetwork'/(Path(archive['name']).stem+'_MN.json')
        if network.exists():
            shutil.copyfile(network, root/'scenario_network.json')
    if live and not (root/'runtime_sources.zip').exists():
        source = cc.ac.ROOT
        assets = source/'autodrive/student_v3_claude'     # the E2E launch's code (rule-based student_v3 removed)
        files = [p for p in assets.rglob('*.py') if not p.name.startswith('test_') and '__pycache__' not in p.parts]
        files += [p for p in assets.glob('*.json') if p.stat().st_size < 10*1024*1024]
        files += [source/'autodrive'/name for name in ['student_v3_follow_claude.sh', 'semantic_fusion.py',
                                                     'lidar_vehicle_detection.py', 'competition_profiles.py']]
        hashes = {}
        with zipfile.ZipFile(str(root/'runtime_sources.zip'), 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=1) as output:
            for path in files:
                if path.is_file():
                    data = path.read_bytes()
                    name = str(path.relative_to(source))
                    output.writestr(name, data)
                    hashes[name] = hashlib.sha256(data).hexdigest()
        cc.atomic_json(root/'runtime_sources.json', dict(captured_wall_ns=time.time_ns(), sha256=hashes,
                       model_weights='Original checkpoint path and hashes are in run.json and versions.md5; weights are not duplicated.'))


def finish_round(index, run_path):
    """Audit indexed bags off the driving thread. Never delete or modify originals."""
    import rosbag
    import rospy
    root = evidence_dir(index)
    root.mkdir(parents=True, exist_ok=True)
    run = Path(run_path).resolve()
    if run.parent != (cc.ac.DATA/'autodrive/follow_runs').resolve() or not run.name.startswith('follow-'):
        raise ValueError('Invalid evidence run path')
    snapshot_scenario(index)
    records, topics, ranges = [], collections.Counter(), []
    bags = sorted(run.glob('*.bag'))
    for path in bags:
        with rosbag.Bag(str(path)) as bag:
            info = bag.get_type_and_topic_info().topics
            for topic, detail in info.items():
                topics[topic] += detail.message_count
            if bag.get_message_count():
                ranges.append((bag.get_start_time(), bag.get_end_time()))
            for _, msg, stamp in bag.read_messages(topics=['/CollisionData']):
                if msg.collision_object:
                    records.append(dict(time=stamp.to_sec(), objects=message_dict(msg.collision_object)))
    episodes = collision_episodes(records)
    camera = read_json(root/'camera_capture.json', {})
    live_events = camera.get('incidents', [])
    for idx, event in enumerate(episodes, 1):
        event.update(id=idx, window_start=event['first_time']-PRE_SECONDS, window_end=event['last_time']+POST_SECONDS)
        contexts = {}
        clip = root/'collision_{:04d}.bag'.format(idx)
        tmp = clip.with_suffix('.bag.tmp')
        counts = collections.Counter()
        with rosbag.Bag(str(tmp), 'w', compression='lz4') as output:
            for path in bags:
                with rosbag.Bag(str(path)) as bag:
                    for topic, raw, stamp, header in bag.read_messages(start_time=rospy.Time.from_sec(event['window_start']),
                                                                     end_time=rospy.Time.from_sec(event['window_end']),
                                                                     raw=True, return_connection_header=True):
                        output.write(topic, raw, stamp, raw=True, connection_header=header)
                        counts[topic] += 1
                    for topic, msg, stamp in bag.read_messages(topics=CONTEXT_TOPICS,
                                                             start_time=rospy.Time.from_sec(event['first_time']-1),
                                                             end_time=rospy.Time.from_sec(event['first_time'])):
                        contexts[topic] = dict(time=stamp.to_sec(), value=message_dict(msg))
        tmp.replace(clip)
        event.update(bag=clip.name, topics=dict(counts), context=contexts)
        ego = contexts.get('/localization/ego_status', {}).get('value', {})
        velocity = ego.get('velocity', {})
        event['speed_kmh'] = round(math.sqrt(sum(velocity.get(k, 0)**2 for k in ('x', 'y', 'z')))*3.6, 2) if velocity else None
        event['position'] = ego.get('position')
        match = next((e for e in live_events if abs(e['first_time']-event['first_time']) < 1), None)
        folder = root/'incident_{:04d}'.format(match['id']) if match else None
        rows = []
        if folder and (folder/'index.jsonl').exists():
            rows = [json.loads(line) for line in (folder/'index.jsonl').read_text().splitlines()]
        event['camera'] = dict(available=bool(rows), folder=folder.name if folder and rows else None,
                               samples=len(rows), first_time=min([r['time'] for r in rows], default=None),
                               last_time=max([r['time'] for r in rows], default=None),
                               reason=None if rows else '주행 당시 카메라 수집 없음 또는 해당 구간 프레임 누락')
        event['recording_window'] = dict(requested_pre_s=PRE_SECONDS, requested_post_s=POST_SECONDS,
                                        bag_start=max(event['window_start'], min([r[0] for r in ranges], default=event['first_time'])),
                                        bag_end=min(event['window_end'], max([r[1] for r in ranges], default=event['last_time'])))
    result = read_json(cc.STATE/'round_{:04d}_result.json'.format(index), {})
    if result:
        cc.atomic_json(root/'result.json', result)
    cc.atomic_json(root/'collisions.json', dict(collision_messages=len(records), incident_count=len(episodes), incidents=episodes))
    files = [dict(path='run/'+str(p.relative_to(run)), bytes=p.stat().st_size) for p in sorted(run.rglob('*')) if p.is_file() and not p.is_symlink()]
    files += [dict(path='evidence/'+str(p.relative_to(root)), bytes=p.stat().st_size) for p in sorted(root.rglob('*'))
              if p.is_file() and not p.is_symlink() and p.name != 'manifest.json']
    manifest = dict(version=1, state='ready', round=index, generated_at=time.time(), run_name=run.name,
                    outcome=result.get('outcome'), scenario=read_json(root/'scenario_config.json', {}).get('name'),
                    collision_messages=len(records), incident_count=len(episodes), incidents=episodes,
                    topics=dict(topics), missing_topics=[t for t in REQUIRED_TOPICS if not topics[t]],
                    camera_capture=camera, camera_recording_enabled=bool(camera),
                    incomplete_bags=[p.name for p in run.glob('*.bag.active')],
                    files=files, total_bytes=sum(f['bytes'] for f in files),
                    note='Full-run bags and logs are unchanged. Camera/semantic evidence is sampled from the live inference feed at up to 15 Hz; gaps and missing historical frames are reported.')
    cc.atomic_json(root/'manifest.json', manifest)
    print(json.dumps(dict(event='evidence_ready', round=index, incidents=len(episodes), camera=bool(camera), bytes=manifest['total_bytes'])), flush=True)
    return manifest


def main():
    import rospy
    from morai_msgs.msg import CollisionData
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    rospy.init_node('competition_evidence', disable_signals=True)
    current = None
    current_idx = None
    current_run = None
    submitted = set()
    lock = threading.Lock()
    pool = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    futures = []

    def on_collision(msg):
        if not msg.collision_object:
            return
        with lock:
            if current and not current.closed:
                current.contact(message_dict(msg.collision_object), time.time(), rospy.Time.now().to_sec())

    rospy.Subscriber('/CollisionData', CollisionData, on_collision, queue_size=100)

    def finalize(index, run, capture=None):
        if capture:
            capture.close()
        # The cycle writes its verdict just after closing its bags.
        deadline = time.monotonic()+30
        while not (cc.STATE/'round_{:04d}_result.json'.format(index)).exists() and time.monotonic() < deadline and not stop.is_set():
            time.sleep(.2)
        return finish_round(index, run)

    try:
        while not stop.is_set():
            state = read_json(cc.STATE/'status.json', {})
            index, phase = state.get('round'), state.get('phase')
            active = index and phase in ('loading', 'starting_drive', 'driving') and time.time()-state.get('updated_at', 0) < 30
            if current and (not active or index != current_idx):
                with lock:
                    finished_capture, current = current, None
                if current_run:
                    submitted.add(current_idx)
                    futures.append(pool.submit(finalize, current_idx, current_run, finished_capture))
                else:
                    futures.append(pool.submit(finished_capture.close))
                current_idx = current_run = None
            if active and current is None and index not in submitted:
                snapshot_scenario(index, live=True)
                with lock:
                    current = CameraWindow(index)
                current_idx = index
                print(json.dumps(dict(event='camera_window_started', round=index)), flush=True)
            if current:
                if phase == 'driving' and state.get('run_path'):
                    current_run = state['run_path']
                current.sample()
            # Backfill old bags; their missing images are explicitly marked.
            for path in sorted(cc.STATE.glob('round_*_result.json')):
                old_index = int(path.name.split('_')[1])
                if old_index in submitted or old_index == current_idx or (evidence_dir(old_index)/'manifest.json').exists():
                    continue
                result = read_json(path, {})
                if result.get('run_path'):
                    submitted.add(old_index)
                    futures.append(pool.submit(finalize, old_index, result['run_path']))
            for future in list(futures):
                if future.done():
                    try:
                        future.result()
                    except Exception as error:
                        print(json.dumps(dict(event='evidence_error', error=str(error))), flush=True)
                    futures.remove(future)
            stop.wait(1/POLL_HZ)
    finally:
        if current:
            current.close()
        pool.shutdown(wait=True)
        rospy.signal_shutdown('Evidence service stopped')


if __name__ == '__main__':
    main()
