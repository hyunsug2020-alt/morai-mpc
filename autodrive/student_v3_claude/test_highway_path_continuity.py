"""Highway publication continuity and retained-prefix safety regressions."""
import math
import unittest

import numpy as np

from highway_planner import HighwayPlanner, local_points, world_points
from highway_three_lane import HighwayLane, ThreeLanePolicy
from planning.highway.path_continuity import (
    join_reference, projected_suffix, published_reference)


def curve(kappa=0., length=300.):
    s = np.arange(0., length+.1, .5)
    if not kappa:
        return np.column_stack((s, s*0.))
    return np.column_stack((np.sin(kappa*s)/kappa,
                            (1.-np.cos(kappa*s))/kappa))


def follower_curvature(path):
    """The wire consumer's central-heading differences, including endpoints."""
    delta = path[2:]-path[:-2]
    h = np.r_[math.atan2(*(path[1]-path[0])[::-1]),
              np.arctan2(delta[:, 1], delta[:, 0]),
              math.atan2(*(path[-1]-path[-2])[::-1])]
    lo = np.maximum(0, np.arange(len(path))-1)
    hi = np.minimum(len(path)-1, np.arange(len(path))+1)
    dh = (h[hi]-h[lo]+np.pi)%(2.*np.pi)-np.pi
    return dh/np.linalg.norm(path[hi]-path[lo], axis=1)


class PublishedReferenceTests(unittest.TestCase):
    def setUp(self):
        self.ego = dict(x=0., y=0., heading_deg=0., stamp=100.)
        self.command = dict(mode='HIGHWAY', gear='D', emergency=False,
            target_mps=28., source_stamp=99.8, received=10., world_path=curve(),
            validation=dict(valid=True, pose_valid=True, steering_valid=True, blocked=False))

    def get(self, command=None, ego=None):
        return published_reference(self.command if command is None else command,
            self.ego if ego is None else ego, 10.2, local_points)

    def test_recent_highway_forward_reference_is_copied(self):
        result = self.get()
        self.assertIsNotNone(result)
        result[0] = 100.
        np.testing.assert_array_equal(self.command['world_path'][0], [0., 0.])

    def test_stop_received_renewal_does_not_renew_valid_geometry(self):
        for changes in (dict(emergency=True), dict(validation=dict(valid=False)),
                        dict(validation='invalid'), dict(validation=[True]),
                        dict(target_mps=float('nan')),
                        dict(source_stamp=98., received=10.19), dict(mode='TUNNEL'),
                        dict(gear='R'), dict(target_mps=0.)):
            with self.subTest(changes=changes):
                self.assertIsNone(self.get(dict(self.command, **changes)))

    def test_time_domains_rewind_future_and_nonfinite_are_rejected(self):
        for changes in (dict(source_stamp=101.), dict(received=11.),
                        dict(received=8.), dict(source_stamp=float('nan')),
                        dict(received=float('inf'))):
            with self.subTest(changes=changes):
                self.assertIsNone(self.get(dict(self.command, **changes)))

    def test_wrong_lane_heading_and_invalid_geometry_are_rejected(self):
        for path in (curve()+[0., 2.], curve()[:, ::-1],
                     np.zeros((8, 2)), np.full((8, 2), np.nan)):
            with self.subTest(path=path[:1].tolist()):
                self.assertIsNone(self.get(dict(self.command, world_path=path)))

    def test_segment_projection_does_not_quantize_progress_to_vertex(self):
        points = curve()+[-.249, .02]
        suffix = projected_suffix(points)
        np.testing.assert_allclose(suffix[0], [0., .02], atol=1e-12)
        self.assertGreater(suffix[1, 0], 0.)


class HighwayJoinGeometryTests(unittest.TestCase):
    def test_published_lookahead_keeps_signed_turn_when_new_escape_reverses_it(self):
        # At 100 km/h, a 0.3 s decision traverses 8.3 m. Matching only the
        # source heading used to reverse the turn by that receipt position.
        for sign in (-1., 1.):
            old = curve(sign*.004, 180.)
            candidate = curve(-sign*.004, 180.)
            path, info = join_reference(candidate, old, 100./3.6, .8)
            wire = path.astype(np.float32).astype(float)
            s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(wire, axis=0), axis=1))]
            at = np.argmin(abs(s-8.3))
            self.assertGreater(sign*follower_curvature(wire)[at], .003)
            self.assertGreater(info['prefix_m'], 20.)
            np.testing.assert_array_equal(path[-1], candidate[-1])

    def test_axle_heading_correction_has_no_lateral_origin_segment(self):
        for offset in (-.35, .35):
            old = curve(.003, 150.)+[0., offset]
            path, _ = join_reference(curve(0., 150.), old, 28.)
            np.testing.assert_array_equal(path[0], [0., 0.])
            self.assertLess(abs(math.atan2(*(path[1]-path[0])[::-1])), .003)
            self.assertTrue(np.all(np.diff(path[:100, 0]) > 0.))

    def test_float32_wire_endpoint_curvature_matches_follower_contract(self):
        reference = curve(.004, 200.)
        path, _ = join_reference(curve(-.004, 200.), reference, 28.)
        kappa = follower_curvature(path.astype(np.float32).astype(float))
        # The consumer uses a one-sided heading at vertex 0 and central
        # headings thereafter, so the first curvature is approximately half.
        self.assertAlmostEqual(kappa[0], .002, delta=.0002)
        self.assertAlmostEqual(kappa[4], .004, delta=.0002)

    def test_reflection_preserves_join_and_input_arrays(self):
        old, new = curve(.004, 180.), curve(-.003, 180.)
        before_old, before_new = old.copy(), new.copy()
        result, _ = join_reference(new, old, 28.)
        reflected, _ = join_reference(new*[1., -1.], old*[1., -1.], 28.)
        np.testing.assert_allclose(reflected, result*[1., -1.], atol=1e-12)
        np.testing.assert_array_equal(old, before_old)
        np.testing.assert_array_equal(new, before_new)

    def test_replanning_does_not_move_the_saved_destination(self):
        saved = curve(.001, 500.)
        published = saved.copy()
        for distance in np.arange(1., 81., 2.):
            p = np.array([math.sin(.001*distance)/.001,
                          (1.-math.cos(.001*distance))/.001])
            ego = dict(x=p[0], y=p[1]+.02, heading_deg=math.degrees(.001*distance)+.03)
            reference = projected_suffix(local_points(published, ego))
            candidate = projected_suffix(local_points(saved, ego))
            joined, _ = join_reference(candidate, reference, 28.)
            published = world_points(joined, ego)
            np.testing.assert_allclose(published[-1], saved[-1], atol=1e-10)
            self.assertLess(np.min(np.linalg.norm(published-(p+[0., .02]), axis=1)), 1e-9)

    def test_insufficient_reference_and_invalid_timing_are_not_silent_zero_steering(self):
        for ref, speed, scale in ((curve(0., 20.), 28., 1.),
                                  (curve(), float('nan'), 1.), (curve(), 28., 0.)):
            with self.subTest(speed=speed, scale=scale):
                with self.assertRaises(ValueError):
                    join_reference(curve(), ref, speed, scale)


class HighwayJoinedAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.route = curve(0., 1000.)
        self.planner = HighwayPlanner(dict(from_idx=0, to_idx=1800, kmh=120.,
            lane_change_policy='free_three_lane', comfort_decel_mps2=2.5,
            comfort_jerk_mps3=4.), self.route)
        self.policy = ThreeLanePolicy(self.planner.config)
        self.policy.continuation_reference = curve(0., 300.)
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=100., stamp=100.)
        self.lane = HighwayLane('1', 1, self.route+[0., 3.5], ('1',), 800.)
        self.path = self.policy.sequence_path([self.lane], 100., 100./3.6)

    def evaluate(self, static, objects=(), map_clearance=None):
        self.planner.longitudinal.begin(self.ego)
        return self.policy.evaluate(self.planner, self.path, self.ego, list(objects),
            map_clearance or (lambda p, h: h), static, changing=True,
            destinations=[self.lane], transition=100.)

    def test_retained_prefix_obstacle_is_not_ignored_for_clear_target_lane(self):
        seen = []
        def static(path, horizon):
            seen.append(path.copy())
            return 12.
        result, check = self.evaluate(static)
        self.assertIsNone(result)
        self.assertTrue(check['path_continuity']['applied'])
        self.assertTrue(seen)
        self.assertLess(np.max(abs(seen[0][:30, 1])), 1e-6)

    def test_map_checks_see_the_joined_candidate(self):
        result, check = self.evaluate(lambda p, h: h, map_clearance=lambda p, h: 20.)
        self.assertIsNone(result)
        self.assertEqual(check['reason'], 'lane change cannot finish inside mapped road')
        self.assertTrue(check['path_continuity']['applied'])

    def test_dynamic_front_contact_cannot_be_released_by_reference_retention(self):
        obstacle = (3., 0., 4.9, 'car', 'npc', 0., 4.9, 2.1, 0.,
                    0., 0., 44, 0., None, True, 0.)
        result, check = self.evaluate(lambda p, h: h, [obstacle])
        self.assertIsNone(result)
        self.assertTrue(check['path_continuity']['applied'])
        self.assertEqual(check['gap_risk']['object_id'], 44)

    def test_short_reference_cannot_admit_unjoined_change(self):
        self.policy.continuation_reference = curve(0., 20.)
        result, check = self.evaluate(lambda p, h: h)
        self.assertIsNone(result)
        self.assertFalse(check['path_continuity']['applied'])
        self.assertEqual(check['reason'], 'lane-change continuity join unavailable')

    def test_join_completion_reserve_includes_the_blend_before_native_merge(self):
        self.lane = HighwayLane('1', 1, self.route+[0., 3.5], ('1',), 110.)
        result, check = self.evaluate(lambda p, h: h)
        self.assertIsNone(result)
        self.assertEqual(check['reason'], 'continuity join cannot finish before native merge')
        self.assertGreater(check['transition_m'], 110.)

    def test_legacy_current_lane_hold_does_not_inherit_a_lane_change_prefix(self):
        self.planner.saved_world = self.path.copy()
        self.policy.continuation_reference = self.path.copy()
        self.planner.longitudinal.begin(self.ego)
        result, check = self.policy.evaluate(self.planner, curve(), self.ego, [],
            lambda p, h: h, lambda p, h: h, continuing_change=False)
        self.assertIsNotNone(result)
        self.assertFalse(check['path_continuity']['applied'])
        np.testing.assert_array_equal(result['path'][:, 1], 0.)

    def test_short_continuation_falls_back_to_a_measured_heading_join(self):
        self.policy.continuation_reference = curve(0., 80.)+[0., .2]
        self.planner.saved_world = self.policy.continuation_reference.copy()
        self.planner.longitudinal.begin(self.ego)
        result, check = self.policy.evaluate(self.planner,
            self.policy.continuation_reference.copy(), self.ego, [],
            lambda p, h: h, lambda p, h: h)
        self.assertIsNotNone(result)
        self.assertTrue(check['path_continuity']['legacy_rejoin'])
        np.testing.assert_array_equal(result['path'][0], [0., 0.])
        self.assertLess(abs(math.atan2(*(result['path'][1]-result['path'][0])[::-1])), .001)

    def test_speed_threshold_noise_does_not_toggle_a_committed_join(self):
        self.planner.saved_world = self.path.copy()
        self.planner.continuation_reference = curve(0., 300.)
        self.policy.target_chain = ('1',)
        self.policy.change_chains = [('1',)]
        self.policy.change_started = 100.
        self.policy.change_goal = 120./3.6
        current = HighwayLane('0', 0, self.route, ('0',), 800.)
        enabled = []
        for n, speed in enumerate((14.999, 15.001, 14.999)):
            result = self.policy.decide(self.planner, 0, self.route,
                dict(self.ego, speed_kmh=speed*3.6, stamp=100.+n*.1), [],
                [current, self.lane], lambda p, h: h, lambda p, h: h, True, None)
            enabled.append(result['status']['path_continuity']['applied'])
        self.assertEqual(enabled, [False, True, True])


if __name__ == '__main__':
    unittest.main()
