"""Fallback searches revalidate current geometry before bounded path reuse."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from lane_corridor import LaneCorridor
from lattice_planner import plan as lattice_plan
from tunnel_runtime import TunnelRuntime, TUNNEL_FALLBACK_KMH


class TunnelFallbackCandidateTests(unittest.TestCase):
    def setUp(self):
        self.runtime = TunnelRuntime()
        self.route = np.column_stack((np.arange(0., 60.01, .5), np.zeros(121)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=10.)
        self.corridor = LaneCorridor(self.route, np.full(121, 6.), np.full(121, -6.))
        self.corridor.measured_walls = True

    def search(self, points=(), clock=100., corridor=None):
        with patch('tunnel_runtime.time.monotonic', return_value=clock):
            return self.runtime.fallback_lattice(self.route, self.ego, points,
                corridor=self.corridor if corridor is None else corridor)

    def test_measured_bypass_uses_at_most_five_targets_and_three_lengths(self):
        points = np.array([(30., y) for y in np.arange(-1., 2.01, .2)])
        with patch('lattice_planner.plan', wraps=lattice_plan) as search:
            result = self.search(points)
        self.assertIsNotNone(result)
        self.assertLess(result['target'], -2.)
        self.assertGreaterEqual(result['clear_m'], 40.)
        self.assertLessEqual(result['candidates_evaluated'], 30)
        self.assertFalse(result['full_search_fallback'])
        self.assertEqual(search.call_count, 1)
        options = search.call_args.kwargs
        self.assertLessEqual(len(options['candidate_targets']), 5)
        self.assertLessEqual(len(options['candidate_lengths']), 3)

    def test_fresh_previous_world_path_is_transformed_and_revalidated_first(self):
        self.ego.update(x=10., y=20., heading_deg=90.)
        self.runtime._fallback_lattice_prev = dict(stamp=99.9, target=0.,
            world_path=np.column_stack((10.-self.route[:, 1], 20.+self.route[:, 0])))
        result = self.search()
        self.assertTrue(result['reused'])
        self.assertEqual(result['candidates_evaluated'], 1)
        np.testing.assert_allclose(result['path'], self.route, atol=1e-10)

    def test_current_raw_wall_blocks_the_cache_and_full_search_is_rate_limited(self):
        self.assertIsNotNone(self.search())
        points = np.array([(3., y) for y in np.arange(-6., 6.01, .2)])
        with patch('lattice_planner.plan', wraps=lattice_plan) as search:
            self.assertIsNone(self.search(points, clock=100.1))
            self.assertIsNone(self.search(points, clock=100.2))
            self.assertIsNone(self.search(points, clock=100.61))
        expanded = [call for call in search.call_args_list if 'candidate_targets' not in call.kwargs]
        bounded = [call for call in search.call_args_list if 'candidate_targets' in call.kwargs]
        self.assertEqual(len(bounded), 3)
        self.assertEqual(len(expanded), 2)

    def test_current_measured_narrowing_invalidates_a_cached_route(self):
        self.assertIsNotNone(self.search())
        width = np.where(self.route[:, 0] < 15., 6., .8)
        corridor = LaneCorridor(self.route, width, -width)
        corridor.measured_walls = True
        self.assertIsNone(self.search(clock=100.1, corridor=corridor))

    def test_frozen_recovery_boxes_and_motion_are_checked_before_reuse(self):
        self.assertIsNotNone(self.search())
        walls, motion = [], []
        def clearance(path, horizon):
            walls.append(path.copy())
            return horizon
        def speed(path, horizon):
            lateral = np.interp(25., path[:, 0], path[:, 1])
            motion.append(lateral)
            return 0. if abs(lateral) < 1.5 else TUNNEL_FALLBACK_KMH/3.6
        recovery = dict(objects=[(25., 0., 1., 'car', 'npc', 0., 1., 1., 0.)],
                        ids=[17], clearance=clearance, speed_cap=speed)
        provider = SimpleNamespace(recovery_inputs=Mock(return_value=recovery))
        self.runtime.path_publisher = SimpleNamespace(_safety_provider=provider)
        result = self.search(clock=100.1)
        self.assertIsNotNone(result)
        self.assertFalse(result['reused'])
        self.assertGreater(abs(result['target']), 1.5)
        self.assertGreater(result['speed_cap_mps'], 0.)
        self.assertEqual(result['dynamic_recovery_ids'], [17])
        self.assertTrue(walls)
        self.assertTrue(motion)

    def test_current_motion_stop_cannot_reuse_a_clear_cached_path_at_cruise(self):
        self.assertIsNotNone(self.search())
        speed = Mock(return_value=0.)
        recovery = dict(objects=[], ids=[17], clearance=lambda path, horizon: horizon,
                        speed_cap=speed)
        self.runtime.path_publisher = SimpleNamespace(
            _safety_provider=SimpleNamespace(recovery_inputs=lambda *args, **kwargs: recovery))
        result = self.search(clock=100.1)
        self.assertIsNotNone(result)
        self.assertEqual(result['speed_cap_mps'], 0.)
        self.assertGreater(speed.call_count, 0)

    def test_proposal_failure_does_not_return_a_cached_safe_path(self):
        self.assertIsNotNone(self.search())
        with patch('planning.tunnel.free_space.free_space_candidates', side_effect=ValueError('invalid frame')):
            self.assertIsNone(self.search(clock=100.1))


if __name__ == '__main__':
    unittest.main()
