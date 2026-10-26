"""Timestamp scheduling and bounded topic lookups for live camera collection."""
from collections import deque
import json
from pathlib import Path
import time


class SampleClock:
    """Keep a fixed sampling grid instead of adding each late frame's delay."""
    def __init__(self, interval_ns):
        self.interval_ns = interval_ns
        self.next_ns = None
        self.last_ns = None

    def due(self, timestamp_ns):
        if self.last_ns is not None and timestamp_ns <= self.last_ns:
            return False
        if self.next_ns is not None and timestamp_ns < self.next_ns:
            return False
        self.last_ns = timestamp_ns
        if self.interval_ns:
            if self.next_ns is None:
                self.next_ns = timestamp_ns + self.interval_ns
            else:
                self.next_ns += ((timestamp_ns-self.next_ns)//self.interval_ns+1)*self.interval_ns
        return True


class TopicIndex:
    """Read recent archived messages and choose the closest timestamp per topic."""
    def __init__(self, root, session):
        self.root = Path(root).resolve() if root else None
        self.session = session
        self.archive = None
        self.offset = 0
        self.checked_at = 0
        self.rows = {topic: deque(maxlen=256) for topic in ('/Object_topic', '/Ego_topic')}

    def refresh(self):
        if self.root is None:
            return
        now = time.monotonic()
        if now-self.checked_at >= .2 or self.archive is None:
            latest = json.loads((self.root/'latest.json').read_text())
            paths = [r.get('archive') for r in latest.values() if isinstance(r,dict) and self.session in r.get('sessions',[]) and r.get('archive')]
            if not paths:
                return
            path = Path(paths[0]).resolve()
            if not path.is_relative_to(self.root):
                raise ValueError('topic archive escapes telemetry store')
            if path != self.archive:
                self.archive, self.offset = path, 0
                for rows in self.rows.values():rows.clear()
            self.checked_at = now
        if self.archive is None:
            return
        with self.archive.open('rb') as stream:
            size = self.archive.stat().st_size
            if self.offset > size:
                self.offset = 0
                for rows in self.rows.values():rows.clear()
            # Bound startup/catch-up work so a long archive cannot stall image capture.
            if size-self.offset > 2*1024**2:
                stream.seek(size-2*1024**2)
                stream.readline()
                self.offset=stream.tell()
            stream.seek(self.offset)
            while stream.tell() < size:
                offset=stream.tell();line=stream.readline(size-offset)
                if not line.endswith(b'\n'):
                    break
                self.offset=stream.tell()
                try:row=json.loads(line)
                except (ValueError,UnicodeDecodeError):continue
                if row.get('topic') not in self.rows or self.session not in row.get('sessions',[]):continue
                self.rows[row['topic']].append({k:row.get(k) for k in ('timestamp_ns','received_at')} | {'archive':str(self.archive),'offset':offset})

    def refs(self, timestamp_ns):
        if self.root is None:
            return {'status':'unavailable'}
        try:self.refresh()
        except (OSError,ValueError,TypeError):pass
        now=time.time();result={}
        for topic,rows in self.rows.items():
            candidates=[r for r in rows if r.get('timestamp_ns') and r.get('received_at') and 0 <= now-r['received_at'] < 1]
            row=min(candidates,key=lambda r:abs(r['timestamp_ns']-timestamp_ns)) if candidates else None
            delta=abs(row['timestamp_ns']-timestamp_ns)/1e6 if row else None
            result[topic]={'status':'timestamp_candidate' if delta is not None and delta<=60 else 'unmatched',
                           'delta_ms':delta,'archive':row['archive'] if row else None,'offset':row['offset'] if row else None,
                           'time_basis_verified':False,'selection':'nearest_archived_timestamp'}
        return result


class PendingRecords:
    """Let delayed ROS messages arrive while image reception continues immediately."""
    def __init__(self, topic_index, delay=.3):
        self.index=topic_index
        self.delay=delay if topic_index.root else 0
        self.rows=deque()

    def append(self, record):
        self.rows.append((time.monotonic()+self.delay,record))

    def flush(self, stream, matches, force=False):
        if force and self.rows:
            time.sleep(max(0,self.rows[-1][0]-time.monotonic()))
        now=time.monotonic()
        while self.rows and (force or self.rows[0][0]<=now):
            _,record=self.rows.popleft()
            record['telemetry']=self.index.refs(record['rgb_timestamp_ns'])
            stream.write(json.dumps(record,ensure_ascii=False)+'\n')
            for topic,ref in record['telemetry'].items():
                if isinstance(ref,dict) and ref.get('status')=='timestamp_candidate':matches[record['view']][topic]+=1
