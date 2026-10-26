"""Bounded tunnel proposals and reuse keep every final safety check."""
import math
import unittest

import numpy as np

from lane_corridor import LaneCorridor
from lattice_planner import offset_profile, plan
from planning.tunnel.free_space import free_space_candidates


class TunnelFreeSpaceTests(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 60.01, .5), np.zeros(121)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=10.)
        self.corridor = LaneCorridor(self.route, np.full(121, 6.), np.full(121, -6.))

    def bounded_plan(self, points=(), objects=(), previous=None, **kwargs):
        spaces = free_space_candidates(self.route, self.ego, points, self.corridor,
                                       objects=objects)
        result = plan(self.route, self.ego, points, objects, self.corridor, previous,
                      cruise_mps=5., outside_max_m=0., max_offset_m=5.,
                      candidate_targets=spaces['targets'], candidate_lengths=spaces['lengths'],
                      candidate_quality=spaces['quality'], reuse_first=True,
                      reuse_min_quality=spaces['reuse_min_quality'], **kwargs)
        return result, spaces

    def test_wider_connected_passage_wins_in_a_small_search(self):
        points = np.array([(30., y) for y in np.arange(-1., 2.01, .2)])
        result, spaces = self.bounded_plan(points)
        self.assertLess(result['target'], -2.)
        self.assertFalse(result['blocked'])
        self.assertEqual(result['clear_m'], result['horizon_m'])
        self.assertLessEqual(len(spaces['targets']), 5)
        self.assertLessEqual(len(spaces['lengths']), 3)
        self.assertLessEqual(result['evaluated'], 30)

    def test_disconnected_wide_pocket_is_not_a_complete_passage(self):
        left, right = np.full(121, 6.), np.full(121, -1.3)
        left[:3] = 1.3
        corridor = LaneCorridor(self.route, left, right)
        points = np.array([(5., y) for y in np.arange(-1., 3.01, .1)])
        spaces = free_space_candidates(self.route, self.ego, points, corridor)
        self.assertFalse(any(p['complete'] for p in spaces['passages']))
        self.assertFalse(any(p['target'] > 3. for p in spaces['passages']))

    def test_tight_offset_gap_keeps_its_midpoint_before_edge_deduplication(self):
        ego = dict(self.ego, speed_kmh=0.)
        sides = np.r_[np.arange(-6., -.9, .1), -.9, 1.4, np.arange(1.5, 6.01, .1)]
        points = np.array([(x, y) for x in np.arange(8., 23., .5) for y in sides])
        spaces = free_space_candidates(self.route, ego, points, self.corridor,
                                       side_margin_m=.12, edge_margin_m=.12)
        result = plan(self.route, ego, points, [], self.corridor,
                      side_margin_m=.12, edge_margin_m=.12, cruise_mps=5./3.6,
                      candidate_targets=spaces['targets'], candidate_lengths=spaces['lengths'],
                      candidate_quality=spaces['quality'], reuse_first=True)
        self.assertFalse(result['blocked'])
        self.assertGreater(result['target'], .1)
        self.assertLess(result['target'], .4)
        self.assertEqual(result['clear_m'], result['horizon_m'])

    def test_frozen_world_box_uses_the_current_ego_frame(self):
        local_box = (25., 2., 2., 'box', 'obstacle', 0., 2., 2., 0.)
        world_box = (8., 45., 2., 'box', 'obstacle', 0., 2., 2., 90.)
        local = free_space_candidates(self.route, self.ego, [], self.corridor,
                                      objects=[local_box])
        world_ego = dict(self.ego, x=10., y=20., heading_deg=90.)
        world = free_space_candidates(self.route, world_ego, [], self.corridor,
                                      objects=[world_box])
        np.testing.assert_allclose(local['targets'], world['targets'], atol=1e-9)
        self.assertLess(local['passages'][0]['target'], 0.)
        self.assertTrue(local['passages'][0]['complete'])

    def test_reuse_transforms_world_path_and_checks_wall_and_dynamic_pace(self):
        self.ego.update(x=100., y=-30., heading_deg=37.)
        yaw = math.radians(self.ego['heading_deg'])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw), math.cos(yaw)]])
        previous = dict(target=0., world_path=self.route @ rotation.T + [100., -30.])
        walls, pace = [], []
        def checked_wall(path, horizon):
            walls.append(path.copy())
            return horizon
        def checked_pace(path, horizon):
            pace.append(path.copy())
            return 5.
        result, _ = self.bounded_plan(previous=previous, extra_clearance=checked_wall,
                                      extra_speed_cap=checked_pace)
        self.assertTrue(result['reused'])
        self.assertEqual(result['evaluated'], 1)
        np.testing.assert_allclose(result['path'], self.route, atol=1e-10)
        self.assertEqual(len(walls), 1)
        self.assertEqual(len(pace), 1)

    def test_new_raw_obstacle_invalidates_a_previous_full_path(self):
        previous = dict(target=0., world_path=self.route.copy())
        points = np.array([(25., y) for y in np.arange(-.4, .41, .2)])
        result, _ = self.bounded_plan(points, previous=previous)
        self.assertFalse(result['reused'])
        self.assertGreater(result['rejected']['previous_collision'], 0)
        self.assertGreater(abs(result['target']), 1.)
        self.assertEqual(result['clear_m'], result['horizon_m'])

    def test_short_previous_path_cannot_claim_an_unchecked_horizon(self):
        previous = dict(target=0., world_path=self.route[:41].copy())
        result, _ = self.bounded_plan(previous=previous)
        self.assertFalse(result['reused'])
        self.assertGreater(result['rejected']['previous_collision'], 0)
        self.assertGreater(np.linalg.norm(np.diff(result['path'], axis=0), axis=1).sum(), 40.)

    def test_narrow_previous_path_does_not_hide_a_wider_current_passage(self):
        lateral = offset_profile(self.route[:, 0], 0., 2., 18.)
        previous = dict(target=2., world_path=self.route + np.column_stack((np.zeros(121), lateral)))
        result, _ = self.bounded_plan(previous=previous)
        self.assertFalse(result['reused'])
        self.assertEqual(result['target'], 0.)
        self.assertGreater(result['evaluated'], 1)

    def test_dynamic_stop_invalidates_reuse_without_a_wall_callback(self):
        previous = dict(target=0., world_path=self.route.copy())
        checked = []
        def dynamic_pace(path, horizon):
            lateral = np.interp(25., path[:, 0], path[:, 1])
            checked.append(lateral)
            return 0. if abs(lateral) < .6 else 5.
        result = plan(self.route, self.ego, [], [], self.corridor, previous,
                      cruise_mps=5., outside_max_m=0., candidate_targets=(-3., 0., 3.),
                      candidate_lengths=(12., 18.), candidate_quality=lambda path, horizon: 3.,
                      reuse_first=True, extra_speed_cap=dynamic_pace)
        self.assertFalse(result['reused'])
        self.assertGreater(abs(result['target']), .6)
        self.assertGreater(result['speed_cap_mps'], 0.)
        self.assertGreater(len(checked), 1)

    def test_every_ranked_proposal_has_wall_and_dynamic_checks(self):
        walls, pace, ranked = set(), set(), set()
        def wall(path, horizon):
            walls.add(id(path))
            return horizon
        def dynamic(path, horizon):
            pace.add(id(path))
            return 5.
        def quality(path, horizon):
            ranked.add(id(path))
            return 2.
        result = plan(self.route, self.ego, [[30., 0.]], [], self.corridor,
                      cruise_mps=5., candidate_targets=(-3., 0., 3.), candidate_lengths=(12., 18.),
                      candidate_quality=quality, reuse_first=True,
                      extra_clearance=wall, extra_speed_cap=dynamic)
        self.assertFalse(result['blocked'])
        self.assertTrue(ranked)
        self.assertLessEqual(ranked, walls & pace)

    def test_wall_callback_failure_stops_a_reused_route(self):
        previous = dict(target=0., world_path=self.route.copy())
        result, _ = self.bounded_plan(previous=previous,
                                      extra_clearance=lambda path, horizon: float('nan'))
        self.assertFalse(result['reused'])
        self.assertTrue(result['blocked'])
        self.assertEqual(result['speed_cap_mps'], 0.)

    def test_beside_body_return_is_kept_in_the_final_sweep(self):
        points = [[2.7, 1.1]]
        spaces = free_space_candidates(self.route, self.ego, points, self.corridor)
        self.assertTrue(spaces['passages'][0]['complete'])
        result = plan(self.route, self.ego, points, [], self.corridor,
                      cruise_mps=5., candidate_targets=(2.,), candidate_lengths=(12.,), reuse_first=True)
        self.assertEqual(result['near_obstacle_points'], 1)
        self.assertTrue(result['blocked'])
        self.assertLess(result['clear_m'], 3.)

    def test_all_curvature_rejections_do_not_publish_an_unchecked_route(self):
        result = plan(self.route, self.ego, [], [], self.corridor,
                      cruise_mps=5., max_offset_m=5., max_curvature=.01,
                      candidate_targets=(5.,), candidate_lengths=(1.,), reuse_first=True)
        self.assertGreater(result['rejected']['curvature'], 0)
        self.assertTrue(result['blocked'])
        self.assertEqual(result['clear_m'], 0.)
        self.assertEqual(result['speed_cap_mps'], 0.)


if __name__ == '__main__':
    unittest.main()
