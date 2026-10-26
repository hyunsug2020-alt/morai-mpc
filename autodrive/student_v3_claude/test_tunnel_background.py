"""M7 tunnel extraction: surveyed wall profile + empty-tunnel voxels + protected road band."""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tunnel_map import (BACKGROUND_FORMAT, LEDGE_CREDIT_M, TunnelBackground, TunnelMap,
                        background_path_for, road_ground)

HERE = Path(__file__).resolve().parent


def synthetic_background(payload, kerb=.5, kerb_top=1.2, occupied=()):
    """Background in the synthetic test frame: a `kerb`-wide kerb up to `kerb_top`
    on both walls, a vertical wall above it and voxels at the given (along, lateral, h)."""
    voxel, along0, lateral0 = .1, -40., -6.
    shape = (int(200/voxel)+1, int(12/voxel)+1, int(4.5/voxel)+1)
    grid = np.zeros(shape, dtype=bool)
    for a, l, h in occupied:
        grid[int(np.floor((a-along0)/voxel)), int(np.floor((l-lateral0)/voxel)), int(np.floor(h/voxel))] = True
    heights = np.arange(int(4.5/.05)+1)*.05
    profile = np.where(heights < kerb_top, kerb, .05)
    frame = list(payload['origin_xy'])+[payload[key] for key in (
        'axis_heading_deg', 'left_offset_m', 'right_offset_m', 'longitudinal_min_m', 'longitudinal_max_m')]
    return dict(format=np.array(BACKGROUND_FORMAT), map_frame=np.asarray(frame, dtype=float),
                grid_bits=np.packbits(grid.ravel()), grid_shape=np.asarray(shape), voxel_m=voxel,
                along_min_m=along0, lateral_min_m=lateral0, profile_bin_m=.05,
                profile_left_m=profile, profile_right_m=profile,
                kerb_face_left_m=kerb, kerb_face_right_m=kerb)


class TunnelBackgroundTests(unittest.TestCase):
    def setUp(self):
        self.payload = dict(format='morai_fixed_tunnel_walls_v1', origin_xy=[0., 0.],
            axis_heading_deg=0., left_offset_m=4., right_offset_m=-4.,
            longitudinal_min_m=0., longitudinal_max_m=100.)
        self.ego = dict(x=20., y=0., heading_deg=0.)
        x, z = np.meshgrid(np.arange(-10., 60.01, .25), [.2, .8, 1.4, 2.0])
        left = np.column_stack((x.ravel(), np.full(x.size, 4.), z.ravel()))
        right = left.copy(); right[:, 1] = -4.
        self.walls = np.vstack((left, right))
        # 0.5 m kerb, 0.8 m high (vehicle frame: ground at -0.4).
        x, z = np.meshgrid(np.arange(-10., 60.01, .1), [-.2, .1, .35])
        kerb_l = np.column_stack((x.ravel(), np.full(x.size, 3.52), z.ravel()))
        kerb_r = kerb_l.copy(); kerb_r[:, 1] = -3.52
        top_l = np.column_stack((x[0], np.full(x.shape[1], 3.75), np.full(x.shape[1], .38)))
        top_r = top_l.copy(); top_r[:, 1] = -3.75
        self.kerbs = np.vstack((kerb_l, kerb_r, top_l, top_r))

    def model(self, **options):
        return TunnelMap(self.payload, TunnelBackground(synthetic_background(self.payload, **options)))

    def test_wall_and_kerb_removed_drums_on_the_road_kept(self):
        model = self.model()
        drum = np.array([[12., 0., .1], [12., .3, .4], [12.3, 2.9, .1], [15., -2.9, .4]])
        result = model.process(np.vstack((self.walls, self.kerbs, drum)), self.ego, [0., 0., -.4])
        self.assertTrue(result['valid'], result['reason'])
        self.assertEqual(result['wall_stats']['extraction'], 'background')
        self.assertGreater(result['wall_stats']['filtered_wall_n'], 1000)
        np.testing.assert_allclose(np.sort(result['obstacle_points'], axis=0), np.sort(drum[:, :2], axis=0))

    def test_planning_faces_use_the_surveyed_kerb_with_the_ledge_credit(self):
        model = self.model(kerb=.5)
        self.assertAlmostEqual(model.inner_left, 4.-(.5-LEDGE_CREDIT_M))
        self.assertAlmostEqual(model.inner_right, -4.+(.5-LEDGE_CREDIT_M))
        model.process(np.vstack((self.walls, self.kerbs)), self.ego, [0., 0., -.4])
        self.assertAlmostEqual(model.inner_left, 4.-(.5-LEDGE_CREDIT_M))   # not re-estimated per scan

    def test_background_voxels_remove_portal_but_never_the_protected_road_band(self):
        portal = (105.05, 1.05, .65)       # beyond the surveyed end: background only
        in_road = (35.05, .05, .65)        # a (wrongly) occupied voxel on the tunnel road
        low_at_kerb = (40.05, 3.25, .2)    # low, 0.75 m inward: outside the protected band
        model = self.model(occupied=(portal, in_road, low_at_kerb))
        ego = self.ego
        points = np.array([[p[0]-ego['x'], p[1], p[2]-.4] for p in (portal, in_road, low_at_kerb)])
        result = model.process(np.vstack((self.walls, points)), ego, [0., 0., -.4])
        self.assertTrue(result['valid'], result['reason'])
        np.testing.assert_allclose(result['obstacle_points'], points[1:2, :2])

    def test_output_keeps_one_real_return_per_tenth_metre_cell(self):
        model = self.model()
        dense = np.column_stack((np.full(9, 12.01), np.linspace(.01, .09, 9), np.full(9, .1)))
        result = model.process(np.vstack((self.walls, dense)), self.ego, [0., 0., -.4])
        self.assertEqual(len(result['obstacle_points']), 1)
        self.assertTrue(np.any(np.all(np.isclose(dense[:, :2], result['obstacle_points'][0]), axis=1)))

    def test_propagated_scan_uses_the_same_survey_with_wider_tolerance(self):
        model = self.model()
        drum = np.array([[12., 0., .1], [14., -2.5, .4]])
        # Kerb seen 0.12 m further inward than surveyed (pose error of the propagation).
        shifted = self.kerbs.copy(); shifted[:, 1] -= np.sign(shifted[:, 1])*.12
        result = model.project_scan(np.vstack((self.walls, shifted, drum)), self.ego, [0., 0., -.4], {'left_n': 9})
        self.assertTrue(result['valid'])
        self.assertTrue(result['wall_stats']['propagated'])
        np.testing.assert_allclose(np.sort(result['obstacle_points'], axis=0), np.sort(drum[:, :2], axis=0))

    def test_missing_corrupt_or_foreign_background_falls_back_to_the_wall_filter(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'map.json'
            path.write_text(json.dumps(self.payload))
            self.assertIsNone(TunnelMap.load(path).background)            # no file
            background_path_for(path).write_bytes(b'not an npz')
            self.assertIsNone(TunnelMap.load(path).background)            # corrupt
            other = dict(self.payload, left_offset_m=4.5)
            np.savez_compressed(background_path_for(path), **synthetic_background(other))
            self.assertIsNone(TunnelMap.load(path).background)            # built for another map
            np.savez_compressed(background_path_for(path), **synthetic_background(self.payload))
            self.assertIsNotNone(TunnelMap.load(path).background)
            legacy = TunnelMap.load(path, background=None)
        self.assertIsNone(legacy.background)
        result = legacy.process(np.vstack((self.walls, [[12., 0., .1]])), self.ego, [0., 0., -.4])
        self.assertTrue(result['valid'])
        self.assertNotIn('extraction', result['wall_stats'])
        np.testing.assert_allclose(result['obstacle_points'], [[12., 0.]])

    def test_g2_road_model_follows_a_longitudinal_vertical_curve(self):
        x, y = np.meshgrid(np.arange(-40., 60., .3), np.arange(-2., 2.01, .3))
        x, y = x.ravel(), y.ravel()
        ground = -.4+.03*x+.12*np.sin(x/12.)                             # 3 % grade + vertical curves
        p = np.column_stack((x, y, ground))
        flat = np.array([0., 0., -.4])
        g = road_ground(p, y, 4., -4., flat)
        self.assertLess(np.percentile(np.abs(p[:, 2]-g), 90), .05)
        self.assertGreater(np.percentile(np.abs(p[:, 2]-(-.4)), 90), .5)     # the fallback plane would not
        np.testing.assert_allclose(road_ground(p[:10], y[:10], 4., -4., flat), -.4)   # unsupported: fallback


class RecordedTunnelBackgroundTests(unittest.TestCase):
    """The shipped survey on recorded scans of the real tunnel."""
    @classmethod
    def setUpClass(cls):
        cls.model = TunnelMap.load(HERE/'tunnel_wall_map.json')
        # The 2026-10-02 capture's poses are in the frame the walls were surveyed in.
        cls.recorded = TunnelMap.load(HERE/'tunnel_wall_map.json', survey_frame=True)

    def test_shipped_survey_is_loaded_for_the_real_map(self):
        self.assertIsNotNone(self.model.background)
        for side in (1, -1):
            self.assertTrue(.3 < self.model.background.kerb_face[side] < .75)
        self.assertLess(self.model.inner_left, self.model.left)
        self.assertGreater(self.model.inner_left, self.model.left-self.model.background.kerb_face[1])

    def test_empty_recorded_tunnel_is_nearly_empty_and_an_injected_drum_survives(self):
        from build_tunnel_map import capture_frames
        root = HERE.parents[1]
        capture = root/'data/autodrive/follow_runs/follow-20261002T152952Z-1309248/tunnel_analysis/tunnel_scans.npz'
        legacy = TunnelMap.load(HERE/'tunnel_wall_map.json', background=None, survey_frame=True)
        for scan, points, ego, plane in capture_frames(capture, scan_indices=range(150, 154)):
            result = self.recorded.process(points, ego, plane)
            self.assertTrue(result['valid'], (scan, result['reason']))
            old = legacy.process(points, ego, plane)
            self.assertLess(len(result['obstacle_points']), max(20, .05*len(old['obstacle_points'])), scan)
            drum = np.array([[x, y, plane[0]*x+plane[1]*y+plane[2]+h] for x in (14., 14.3)
                             for y in (-.6, -.3, 0.) for h in (.4, .7)])
            result = self.recorded.process(np.vstack((points, drum)), ego, plane)
            near = np.linalg.norm(result['obstacle_points'][:, None]-drum[None, :, :2], axis=2).min(axis=0)
            self.assertLess(near.max(), .15, scan)


if __name__ == '__main__':
    unittest.main()
