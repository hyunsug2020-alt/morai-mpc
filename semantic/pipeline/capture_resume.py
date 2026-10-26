"""Validate a closed capture before appending another collection segment."""
import json


def resume_state(session, expected):
    meta = json.loads((session / 'session.json').read_text())
    if not meta.get('finished_at') or any(meta.get(k) for k in
        ('capture_complete', 'validation_required', 'batch_complete', 'exclude_from_training')):
        raise ValueError('Only a closed, unfinished, unqueued capture can be resumed.')
    for key in ('session_id', 'group', 'camera_pairs', 'save_fps', 'every_frame',
                'sensor_preset_sha256', 'network_preset_sha256', 'sync_tolerance_ms', 'recording_id'):
        if meta.get(key) != expected.get(key):
            raise ValueError('Capture setting changed: ' + key)
    counts = {v: 0 for v in expected['camera_pairs']}
    first = {}; last = {}
    with (session / 'manifest.jsonl').open() as stream:
        for line in stream:
            if not line.endswith('\n'):
                raise ValueError('Capture manifest has an unfinished record.')
            row = json.loads(line); view = row['view']
            if view not in counts or row['sequence'] != counts[view] + 1:
                raise ValueError('Capture sequence is inconsistent.')
            stamp = row['rgb_timestamp_ns']
            if view in last and stamp <= last[view]:
                raise ValueError('Capture timestamps are not increasing.')
            counts[view] += 1; first.setdefault(view, stamp); last[view] = stamp
    if counts != meta.get('saved_per_view') or any(v == 0 for v in counts.values()):
        raise ValueError('Closed capture count does not match the manifest.')
    return meta, counts, first, last
