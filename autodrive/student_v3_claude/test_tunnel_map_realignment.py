"""The tunnel wall map re-aligned to the world (2026-10-10).

The 2026-10-02 survey took its axis from ESKF headings logged before
imu_yaw_variance_override pinned the heading to the IMU. Every later pass measured the
walls 0.544 deg / 0.177 m off the map while the ESKF matched withheld raw GPS, so the
correction grew ~1 m per 100 m, map aiding was refused past 0.8 m and the E2E tunnel
stalled. Scans recorded on 2026-10-08 must match the re-aligned walls closely.
"""
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from tunnel_map import TunnelMap  # noqa: E402

RECORDED = ('tunnel_pose_rejected_20261008.npz', 'tunnel_semantic_stop_20261008.npz',
            'tunnel_wide_pass_entry_20261008.npz')


class RealignedTunnelMap(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from lidar_corridor import LidarCorridor
        from tunnel_runtime import pose_at_stamp
        cls.payload = json.loads((HERE/'tunnel_wall_map.json').read_text())
        cls.current = TunnelMap.load(HERE/'tunnel_wall_map.json')
        cls.survey = TunnelMap.load(HERE/'tunnel_wall_map.json', survey_frame=True)
        cls.scans = {}
        for name in RECORDED:
            data = np.load(HERE/'test_data'/name)
            meta = json.loads(str(data['meta']))
            lidar = LidarCorridor(x_range=(-75., 100.))
            lidar.ingest(data['xyz'], stamp=meta['stamp'])
            snapshot = lidar.scan_snapshot()
            cls.scans[name] = (snapshot['xyz'], snapshot['ground_plane'], pose_at_stamp(meta['poses'], meta['stamp']))

    def test_current_scans_match_the_realigned_walls_and_aid_the_eskf(self):
        for name, (xyz, plane, ego) in self.scans.items():
            with self.subTest(name=name):
                match = self.current.process(xyz, ego, plane)
                self.assertTrue(match['valid'], match['reason'])
                self.assertLess(abs(match['wall_stats']['lateral_correction_m']), .3)
                self.assertLess(abs(match['wall_stats']['yaw_correction_deg']), .15)
                self.assertEqual(match['constraint']['source'], 'tunnel_lidar_wall')
                self.assertEqual(match['wall_stats']['extraction'], 'background')

    def test_the_survey_frame_reproduces_the_recorded_rejections(self):
        for name in RECORDED[:2]:   # 79 m and 94 m into the tunnel
            xyz, plane, ego = self.scans[name]
            match = self.survey.process(xyz, ego, plane)
            self.assertFalse(match['valid'])
            self.assertEqual(match['reason'], 'pose correction exceeds validated limit')
            self.assertGreater(match['wall_stats']['lateral_correction_m'], .8)
            self.assertAlmostEqual(match['wall_stats']['yaw_correction_deg'], .55, delta=.1)

    def test_realignment_is_rigid(self):
        old = self.payload['realigned_from']
        for key in ('left_offset_m', 'right_offset_m', 'longitudinal_min_m', 'longitudinal_max_m'):
            self.assertEqual(self.payload[key], old[key])
        self.assertAlmostEqual(self.payload['axis_heading_deg']-old['axis_heading_deg'], -.544, places=6)
        shift = np.subtract(self.payload['origin_xy'], old['origin_xy'])
        self.assertAlmostEqual(float(np.linalg.norm(shift)), .177, places=6)
        self.assertAlmostEqual(float(shift @ self.survey.axis), 0., places=6)     # lateral only
        for side, offset in (('left', self.current.left), ('right', self.current.right)):
            for point in self.payload['walls'][side]:
                self.assertAlmostEqual(float((np.array(point)-self.current.origin) @ self.current.normal), offset, places=6)

    def test_the_wall_aligned_survey_serves_both_frames_but_no_other_walls(self):
        self.assertIsNotNone(self.current.background)
        self.assertIsNotNone(self.survey.background)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'tunnel_wall_map.json'
            (Path(tmp)/'tunnel_background.npz').write_bytes((HERE/'tunnel_background.npz').read_bytes())
            path.write_text(json.dumps(dict(self.payload, left_offset_m=self.payload['left_offset_m']+.3)))
            self.assertIsNone(TunnelMap.load(path).background)                 # other walls, same history
            path.write_text(json.dumps({k: v for k, v in self.payload.items() if k != 'realigned_from'}))
            self.assertIsNone(TunnelMap.load(path).background)                 # no recorded survey frame
            path.write_text(json.dumps(self.payload))
            self.assertIsNotNone(TunnelMap.load(path).background)


if __name__ == '__main__':
    unittest.main()
