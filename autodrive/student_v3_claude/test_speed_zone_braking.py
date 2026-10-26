"""Speed-limit entries brake before the boundary instead of issuing a pedal step."""
import math
import os
from pathlib import Path
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

import gui
from gui import SpeedZones
from ego_body import EGO_FRONT_M
from stop_line import INTERSECTION_ENTRY_CAP_FRONT_GAP_M


def preview_region(station):
    return dict(stop_station_m=station+EGO_FRONT_M+INTERSECTION_ENTRY_CAP_FRONT_GAP_M)


class SpeedZoneBrakingTests(unittest.TestCase):
    def zones(self, items=()):
        zones = SpeedZones.__new__(SpeedZones)
        zones.base, zones.route_len = 55., 401
        zones.zones, zones.loaded = list(items), time.monotonic()
        zones.file_cache = SimpleNamespace(value=list(items))
        zones.set_route(np.column_stack((np.arange(401) * .5, np.zeros(401))))
        return zones

    def test_dynamic_entry_reduces_speed_before_entry(self):
        zones = self.zones()
        caps = [zones.braking_kmh(i, [(120, 300, 40.)])[0] for i in range(121)]
        self.assertAlmostEqual(caps[0], 55.)
        self.assertGreater(caps[80], 40.)
        self.assertLess(caps[80], 55.)
        self.assertEqual(caps[120], 40.)
        self.assertLess(max(abs(np.diff(caps))), .4)

    def test_next_lower_dynamic_zone_is_also_previewed(self):
        zones = self.zones()
        self.assertLess(zones.braking_kmh(195, [(120, 200, 40.), (201, 300, 30.)])[0], 32.)
        self.assertEqual(zones.braking_kmh(201, [(120, 200, 40.), (201, 300, 30.)])[0], 30.)

    def test_explicit_and_percentage_limits(self):
        for item in (dict(from_idx=120, to_idx=300, kmh=40.),
                     dict(from_pct=12000 / 401, to_pct=75., kmh=40.)):
            zones = self.zones([item])
            self.assertLess(zones.braking_kmh(110)[0], 42.)
            self.assertAlmostEqual(zones.braking_kmh(120)[0], 40.)

    def test_first_matching_zone_keeps_existing_priority(self):
        zones = self.zones([dict(from_idx=0, to_idx=400, kmh=55.),
                            dict(from_idx=120, to_idx=300, kmh=20.)])
        self.assertEqual(zones.braking_kmh(110)[0], 55.)

    def test_preview_wraps_closed_route(self):
        zones = self.zones()
        theta = np.linspace(0., 2. * math.pi, 401)
        zones.set_route(30. * np.column_stack((np.cos(theta), np.sin(theta))))
        self.assertLess(zones.braking_kmh(380, [(0, 100, 30.)])[0], 40.)
        self.assertEqual(zones.braking_kmh(0, [(0, 100, 30.)])[0], 30.)

    def test_preview_stops_acceleration_brake_saturation_in_closed_loop(self):
        zones = self.zones()
        x, v, acceleration = 0., 55. / 3.6, 0.
        brakes = []
        while x < 60.:
            cap = zones.braking_kmh(int(x / .5), [(120, 300, 40.)])[0] / 3.6
            error = cap - v
            brake = min(1., max(0., -.45 * error)) if error < -.15 else 0.
            throttle = min(1., max(0., .35 * error)) if error > .15 else 0.
            acceleration += .05 / .25 * (-8. * brake + 3. * throttle - acceleration)
            v = max(0., v + acceleration * .05)
            x += v * .05
            brakes.append(brake)
        self.assertLess(max(brakes), .95)
        self.assertLessEqual(v * 3.6, 42.)

    def test_junction_preview_reaches_existing_cap_before_the_activation_boundary(self):
        zones = self.zones()
        zones.set_junction_previews([preview_region(60.)], 30.)
        caps = [zones.braking_kmh(i)[0] for i in range(121)]
        self.assertEqual(caps[0], 55.)
        self.assertGreater(caps[80], 30.)
        self.assertLess(caps[80], 55.)
        self.assertAlmostEqual(caps[114], 30.)
        self.assertAlmostEqual(caps[120], 30.)
        self.assertLess(max(abs(np.diff(caps))), .4)
        self.assertEqual(zones.kmh_at(120)[0], 55.)

    def test_preview_bridges_the_boundary_then_releases_or_can_be_disabled(self):
        zones = self.zones()
        zones.set_junction_previews([preview_region(60.)], 30.)
        self.assertAlmostEqual(zones.braking_kmh(121)[0], 30.)
        self.assertAlmostEqual(zones.braking_kmh(126)[0], 30.)
        self.assertEqual(zones.braking_kmh(127)[0], 55.)
        zones.set_junction_previews([], 30.)
        self.assertEqual(zones.braking_kmh(110)[0], 55.)

    def test_multiple_junctions_and_existing_lower_limits_keep_the_lowest_cap(self):
        zones = self.zones([dict(from_idx=100, to_idx=150, kmh=20.)])
        zones.set_junction_previews([preview_region(60.),
                                    preview_region(160.)], 30.)
        self.assertEqual(zones.braking_kmh(110)[0], 20.)
        self.assertAlmostEqual(zones.braking_kmh(320)[0], 30.)
        self.assertEqual(zones.braking_kmh(327)[0], 55.)
        zones.zones, zones.file_cache.value = [], []
        self.assertEqual(zones.braking_kmh(110, [(100, 150, 25.)])[0], 25.)

    def test_junction_preview_wraps_the_same_closed_route(self):
        zones = self.zones()
        theta = np.linspace(0., 2.*math.pi, 401)
        zones.set_route(30.*np.column_stack((np.cos(theta), np.sin(theta))))
        zones.set_junction_previews([preview_region(0.)], 30.)
        self.assertLess(zones.braking_kmh(380)[0], 40.)
        self.assertAlmostEqual(zones.braking_kmh(0)[0], 30.)
        self.assertAlmostEqual(zones.braking_kmh(1)[0], 30.)
        self.assertEqual(zones.braking_kmh(7)[0], 55.)

    def test_invalid_preview_metadata_does_not_create_a_speed_zone(self):
        zones = self.zones()
        zones.set_junction_previews([{}, preview_region(-1.),
            preview_region(math.nan), preview_region(math.inf),
            preview_region(zones.loop_length)], 30.)
        self.assertEqual(zones.braking_kmh(110)[0], 55.)
        zones.set_junction_previews([preview_region(60.)], math.nan)
        self.assertEqual(zones.braking_kmh(110)[0], 55.)

    def test_junction_preview_avoids_a_full_brake_step_in_closed_loop(self):
        zones = self.zones()
        zones.set_junction_previews([preview_region(60.)], 30.)
        x, v, acceleration = 0., 55./3.6, 0.
        brakes = []
        while x < 60.:
            cap = zones.braking_kmh(int(x/.5))[0]/3.6
            error = cap-v
            brake = min(1., max(0., -.45*error)) if error < -.15 else 0.
            throttle = min(1., max(0., .35*error)) if error > .15 else 0.
            acceleration += .05/.25*(-8.*brake+3.*throttle-acceleration)
            v = max(0., v+acceleration*.05)
            x += v*.05
            brakes.append(brake)
        self.assertLess(max(brakes), .95)
        self.assertLessEqual(v*3.6, 32.)

    def test_runtime_connects_only_its_enabled_map_regions_to_the_preview(self):
        root = Path(__file__).resolve().parent
        map_root = (root/'link_set.json').resolve().parent
        with patch.dict(os.environ, STUDENT_HDMAP_ZIP=str(map_root.parent/'ws/src/moraimpc/data/hdmap.zip')):
            lanes = gui.HdMapLanes(root/'link_set.json')
        args = SimpleNamespace(checkpoint='unused', camera_source='files',
            camera_files_dir='unused', ego_topic='unused', publish_control_path=False,
            cruise_kmh=55., route=str(root/'global_path.json'))
        checkpoint = dict(config={}, actions=[], concepts=[], gallery=[dict(signature=[1.])])
        for enabled in (False, True):
            with self.subTest(enabled=enabled), patch('gui.YIELD_AT_JUNCTION', enabled), \
                    patch('gui.load_checkpoint', return_value=(None, checkpoint)), \
                    patch('gui.RecordedCameraReceiver'), patch('gui.RosEgoReceiver'), \
                    patch('gui.RosObjectReceiver'), patch('gui.VehicleReceiver'), \
                    patch('gui.LidarCorridor'), patch('gui.HdMapLanes', return_value=lanes), \
                    patch('gui.AsyncJsonCache', return_value=SimpleNamespace(value=[])), \
                    patch.object(gui.Runtime, 'init_tunnel'), patch.dict(os.environ):
                for name in ('STUDENT_HIGHWAY_MISSION', 'STUDENT_INTERSECTION_PERCEPTION',
                             'STUDENT_V6_CHECKPOINT'):
                    os.environ.pop(name, None)
                runtime = gui.Runtime(args)
                zones = runtime.speed_zones
                zones.base = 58.
                if enabled:
                    self.assertEqual(len(runtime.intersection_regions), 1)
                    start = runtime.intersection_regions[0]['start_station_m']
                    self.assertAlmostEqual(start, 805.1006875597407)
                    self.assertEqual(len(zones.junction_previews), 1)
                    preview = runtime.intersection_regions[0]['stop_station_m']-EGO_FRONT_M-INTERSECTION_ENTRY_CAP_FRONT_GAP_M
                    self.assertAlmostEqual(zones.junction_previews[0][0], preview)
                    self.assertEqual(gui.zone_speed_kmh(zones, 1619)[0], 58.)
                    self.assertAlmostEqual(zones.junction_previews[0][1], 30.)
                    index = int(np.searchsorted(zones.station, preview))
                    caps = [gui.zone_speed_kmh(zones, i)[0] for i in range(index-20, index+1)]
                    self.assertAlmostEqual(caps[-1], 30.)
                    self.assertLess(max(abs(np.diff(caps))), .4)
                    self.assertTrue(runtime.stop_line_controller.yield_only)
                    self.assertEqual(runtime.stop_line_controller.hold_s, 0.)
                    self.assertFalse(runtime.stop_line_controller.released)
                    self.assertFalse(runtime.gap_planner.junction_committed)
                else:
                    self.assertIsNone(runtime.stop_line_controller)
                    self.assertEqual(zones.junction_previews, ())
                    self.assertEqual(gui.zone_speed_kmh(zones, 1619)[0], 58.)

    def test_ahead_route_status_cannot_reopen_speed_before_the_fresh_junction_context(self):
        import json
        import test_live_path_safety as live_fixture
        root = Path(__file__).resolve().parent
        points = np.asarray([[p['x'], p['y']] for p in
            json.loads((root/'global_path.json').read_text())['waypoints']])
        zones = self.zones()
        zones.base, zones.route_len = 58., len(points)
        zones.set_route(points)
        zones.set_junction_previews([dict(stop_station_m=885.1006875597407)], 30.)
        boundary = int(np.searchsorted(zones.station, zones.junction_previews[0][0]))
        fixture = live_fixture.LivePathSafetyTests()
        runtime = fixture.attach(fixture.runtime())
        runtime.speed_zones = zones
        runtime.test_status['ego'].update(x=points[boundary-1, 0], y=points[boundary-1, 1],
            heading_deg=math.degrees(math.atan2(*(points[boundary]-points[boundary-1])[::-1])),
            speed_kmh=30.)
        publisher = runtime.path_publisher
        publisher.cruise_mps = 58./3.6
        publisher._last_target_mps, publisher._last_target_time = 30./3.6, 100.
        # The path status crosses the region before its current-pose traffic
        # context exists. Exercise the actual Runtime and serialized publisher,
        # rather than relying on the entry planner to fill this one-frame gap.
        for index in (boundary-1, boundary, boundary+1, boundary+5):
            runtime.test_status['path']['nearest_idx'] = index
            with self.subTest(index=index), patch('time.monotonic', return_value=100.):
                self.assertTrue(runtime.infer())
                payload = fixture.payload(runtime)
                self.assertAlmostEqual(payload['target_speed_mps']*3.6, 30.)
                self.assertIsNone(runtime.stop_line_controller)
                self.assertIsNone(runtime.gap_planner.junction)


if __name__ == "__main__":
    unittest.main()
