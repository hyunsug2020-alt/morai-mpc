"""A moving tunnel bypass searches for visible room before a stop is needed."""
import unittest
from unittest.mock import Mock, patch

import numpy as np

import gui
import model_avoidance as ma
import test_tunnel_integration as fixture
import tunnel_runtime


class TunnelCoverageAdmissionTests(unittest.TestCase):
    def drive(self, alternative=25., points=(), inside=True, observed_now=17.):
        runtime = fixture.TunnelIntegrationTests().runtime(points, half_width=9.)
        runtime.tunnel.active = lambda ego: inside
        # Native 2026-10-20 failure: 38.225 km/h, observed floor 17 m,
        # clear geometry 58 m. The former fixed 12 m trigger never searched.
        runtime.test_status['ego']['speed_kmh'] = 38.22511120873096
        runtime.test_odometry['speed_kmh'] = 38.22511120873096
        plan_path, peak, _ = ma._geometry.build_forced_bypass(
            runtime.test_route, 1., offset_m=3.5, hold_m=30.,
            max_curvature_1pm=gui.MAX_CONTROL_CURVATURE, transitions=(28.,))
        plan = dict(path=plan_path, kind='bypass', direction=1., offset_m=3.5,
                    transition_m=28., curvature=peak, generator='portable')
        runtime.tunnel_model_static = Mock(return_value=(plan, [], [], None, None, {}))

        def observed(coverage, path, *args, **kwargs):
            return observed_now if np.max(np.asarray(path)[:, 1]) > 3. else alternative

        with patch('tunnel_runtime.TUNNEL_MODEL', True), \
                patch('tunnel_runtime.TUNNEL_LATTICE', False), \
                patch('tunnel_visibility.ObservedLidarCoverage.clear_distance', observed), \
                patch.object(runtime, 'tunnel_coverage_escape', wraps=runtime.tunnel_coverage_escape) as search:
            self.assertTrue(runtime.infer_tunnel(runtime.test_status, runtime.test_status['ego']))
        return runtime, search

    def test_native_speed_searches_before_short_coverage_forces_a_stop(self):
        runtime, search = self.drive()
        search.assert_called_once()
        record = runtime._web_record
        self.assertTrue(record['tunnel']['coverage_escape']['applied'])
        self.assertGreaterEqual(record['tunnel']['observed_clear_distance_m'], 25.)
        self.assertFalse(record['emergency_stop'], record['stop_reason'])
        self.assertGreater(runtime.path_publisher.velocity_pub.items[-1].data, 0.)

    def test_no_visible_alternative_keeps_the_original_coverage_guard(self):
        runtime, search = self.drive(alternative=17.)
        search.assert_called_once()
        self.assertFalse(runtime._web_record['tunnel']['coverage_escape']['applied'])
        self.assertLessEqual(runtime._web_record['planner_cap_kmh'], 38.22511120873096)

    def test_visibility_search_does_not_authorize_a_physical_blocker(self):
        barrier = [(5., y) for y in np.arange(-9., 9., .1)]
        runtime, _ = self.drive(points=barrier)
        self.assertTrue(runtime._web_record['emergency_stop'])
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)

    def test_outside_physical_tunnel_preserves_previous_search_and_acceptance(self):
        runtime, search = self.drive(inside=False)
        search.assert_not_called()
        runtime, search = self.drive(inside=False, observed_now=10., alternative=14.5)
        search.assert_called_once()
        self.assertIsNone(search.call_args.kwargs['required_observed_m'])
        self.assertFalse(runtime._web_record['tunnel']['coverage_escape']['applied'])


if __name__ == '__main__':
    unittest.main()
