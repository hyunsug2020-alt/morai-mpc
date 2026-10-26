import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import competition_evidence as evidence


class EvidenceTests(unittest.TestCase):
    def test_duplicate_contacts_are_one_incident_but_a_later_impact_is_separate(self):
        records = [dict(time=t, objects=[dict(unique_id=20003)]) for t in [100, 100.0002, 100.04, 100.0402, 143]]
        events = evidence.collision_episodes(records)
        self.assertEqual(len(events), 2)
        self.assertEqual(len(events[0]['messages']), 4)
        self.assertEqual(events[0]['object_ids'], ['20003'])

    def test_original_camera_bytes_cover_pre_and_post_contact_window(self):
        with tempfile.TemporaryDirectory() as directory:
            state, feed = Path(directory)/'state', Path(directory)/'feed'
            state.mkdir(); feed.mkdir()
            with patch.object(evidence.cc, 'STATE', state), patch.object(evidence, 'FEED', feed), patch.object(evidence, 'FEED_FILES', ('frames.npz',)):
                camera = evidence.CameraWindow(1)
                base = time.time()
                camera.started_ns = int((base-20)*1e9)
                def sample(offset):
                    path=feed/'frames.npz'
                    path.write_bytes(('original-'+str(offset)).encode())
                    stamp=int((base+offset)*1e9)
                    os.utime(str(path), ns=(stamp, stamp))
                    camera.sample(base+offset)
                sample(-11); sample(-9); sample(-3)
                camera.contact([dict(unique_id=1)], base, base)
                sample(0); sample(4); sample(6)
                camera.close()
                rows=[json.loads(line) for line in (camera.root/'incident_0001/index.jsonl').read_text().splitlines()]
                saved=[(camera.root/r['files'][0]['path']).read_bytes() for r in rows]
                self.assertEqual(saved, [b'original--9', b'original--3', b'original-0', b'original-4'])
                self.assertEqual(json.loads((camera.root/'camera_capture.json').read_text())['state'], 'complete')

    def test_memory_limit_is_bounded_and_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            state, feed=Path(directory)/'state',Path(directory)/'feed'
            state.mkdir();feed.mkdir()
            with patch.object(evidence.cc,'STATE',state),patch.object(evidence,'FEED',feed),patch.object(evidence,'FEED_FILES',('frames.npz',)),patch.object(evidence,'MAX_BUFFER_BYTES',1):
                camera=evidence.CameraWindow(1)
                (feed/'frames.npz').write_bytes(b'larger than buffer')
                stamp=camera.started_ns+1000000
                os.utime(str(feed/'frames.npz'), ns=(stamp, stamp))
                camera.sample(stamp/1e9+.01)
                self.assertEqual(camera.buffer_bytes,0)
                self.assertEqual(camera.memory_trimmed,1)
                camera.close()


if __name__ == '__main__':
    unittest.main()
