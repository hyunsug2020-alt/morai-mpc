"""Road fusion geometry keeps observed surfaces while sharing one margin."""
import math
import unittest

import numpy as np

import model_avoidance as ma
from perception.obstacle_geometry import (FUSION_OBSTACLE_MIN_DIMENSION_M,
                                          FUSION_OBSTACLE_DIMENSION_PADDING_M)
from perception.stationary_semantics import metadata, with_metadata


def obstacle(x=5., y=-1.6, size=(1.1, 1.1), *, name='fusion_obstacle',
             kind='obstacle', speed=0., heading=0.):
    return (x, y, max(size), name, kind, speed, *size, heading, 0., 0., 7)


class RoadStaticFusionTests(unittest.TestCase):
    def setUp(self):
        self.ego = dict(x=0., y=0., heading_deg=0.)
        self.points = np.array([[4.9, -1.8], [4.9, -1.6], [5.1, -1.4]])
        self.obj = obstacle()

    def normalize(self, objects=None, points=None, ego=None):
        return ma.road_static_objects([self.obj] if objects is None else objects,
            self.ego if ego is None else ego, self.points if points is None else points)

    def test_observed_low_box_retains_physical_prior_and_shared_margin(self):
        source = with_metadata(self.obj, raw_size_m=[1.1, 1.1], height_m=.4)
        checked, = self.normalize([source])
        np.testing.assert_allclose(checked[6:8], [.9, .9])
        self.assertAlmostEqual(checked[2], .9)
        self.assertEqual(checked[:2], source[:2])
        self.assertEqual(checked[3:6], source[3:6])
        self.assertEqual(checked[8:17], source[8:17])
        self.assertEqual(source[6:8], (1.1, 1.1))
        geometry = metadata(checked)['road_static_geometry']
        self.assertEqual(geometry['source_size_m'], [1.1, 1.1])
        np.testing.assert_allclose(geometry['physical_size_m'], [.8, .8])
        self.assertAlmostEqual(geometry['applied_padding_m']/2.+ma.PORTABLE_STATIC_MARGIN_M,
                               ma.ROAD_STATIC_MARGIN_M)
        self.assertEqual(metadata(checked)['raw_size_m'], [1.1, 1.1])
        self.assertEqual(metadata(checked)['height_m'], .4)

    def test_repeated_and_memory_calls_never_reduce_geometry_again(self):
        checked, = self.normalize()
        for _ in range(10):
            repeated, = self.normalize([checked])
            self.assertIs(repeated, checked)
        # Rechecking the unchanged original observation is deterministic too.
        again, = self.normalize()
        np.testing.assert_array_equal(again[6:8], checked[6:8])

    def test_source_or_already_corrected_padding_is_not_guessed(self):
        for source in (with_metadata(self.obj, fusion_padding_m=.1),
                       with_metadata(self.obj, fusion_padding_m=0.),
                       with_metadata(self.obj, fusion_padding_m=float('nan')),
                       with_metadata(self.obj, raw_size_m=[1.3, 1.3]),
                       with_metadata(self.obj, road_static_geometry={}),
                       obstacle(size=(.9, .9))):
            with self.subTest(source=source):
                self.assertIs(self.normalize([source])[0], source)

    def test_other_classes_motion_and_invalid_geometry_remain_unchanged(self):
        cases = [obstacle(name='fusion_parked_vehicle', size=(4.6, 1.9)),
                 obstacle(name='fusion_pedestrian', kind='pedestrian'),
                 obstacle(name='fusion_static_candidate'), obstacle(name='lidar_static'),
                 obstacle(kind='npc'), obstacle(speed=.2), obstacle(speed=-.3),
                 obstacle(size=(float('nan'), 1.1)), obstacle(size=(1.1, .3)),
                 obstacle(heading=float('inf')), obstacle(x=float('nan')),
                 (0.,), obstacle(name='fusion_obstacle_custom')]
        for source in cases:
            with self.subTest(source=source):
                self.assertIs(self.normalize([source])[0], source)

    def test_missing_or_unassociated_raw_evidence_keeps_source_box(self):
        for points in ([], np.empty((0, 2)), np.zeros((3, 3)), np.array([[50., 50.]]),
                       np.array([[float('nan'), 0.]])):
            with self.subTest(points=points):
                self.assertIs(self.normalize(points=points)[0], self.obj)
        for ego in ({}, dict(x=0., y=0., heading_deg=float('nan'))):
            self.assertIs(self.normalize(ego=ego)[0], self.obj)
        self.assertIs(ma.road_static_objects([self.obj], self.ego, None)[0], self.obj)

    def test_raw_extents_can_enlarge_prior_but_are_never_cropped(self):
        points = np.array([[5.46, -1.6], [5.1, -1.2]])
        checked, = self.normalize(points=points)
        self.assertAlmostEqual(checked[6], 1.02)
        self.assertGreaterEqual(checked[7], .9-1e-12)
        observed = points-np.array(checked[:2])
        self.assertTrue((np.abs(observed) <= np.array(checked[6:8])/2.-.05+1e-10).all())
        # A raw surface on the padded boundary leaves no certified room to trim.
        edge = np.array([[5.55, -1.6]])
        self.assertIs(self.normalize(points=edge)[0], self.obj)

    def test_real_obstacle_still_blocks_straight_path_with_original_raw_guard(self):
        from lane_corridor import LaneCorridor
        route = np.column_stack((np.arange(0., 30.01, .25), np.zeros(121)))
        corridor = LaneCorridor(route, np.full(121, 3.), np.full(121, -3.))
        points = np.array([[5., -.2], [5., 0.], [5., .2]])
        source = obstacle(y=0.)
        checked, = self.normalize([source], points=points)
        np.testing.assert_allclose(checked[6:8], [.9, .9])
        _, clear = ma.road_static_checks(corridor, points)
        self.assertLess(clear(route, 30.), 2.)
        self.assertEqual(corridor.body_shift_m, 0.)

    def test_large_oblique_boxes_keep_all_observed_surfaces_and_prior(self):
        rng = np.random.default_rng(731)
        for ego_heading in (0., 38., 179.):
            for object_heading in (-52., 0., 65., 143.):
                observed_extent = np.array([2.4, .5])
                physical = np.maximum(observed_extent, FUSION_OBSTACLE_MIN_DIMENSION_M)
                source_size = physical+FUSION_OBSTACLE_DIMENSION_PADDING_M
                yaw = math.radians(object_heading)
                axes = np.array([[math.cos(yaw), -math.sin(yaw)],
                                 [math.sin(yaw), math.cos(yaw)]])
                corners = np.array([[-1., -1.], [-1., 1.], [1., -1.], [1., 1.]])
                local = np.vstack((corners, rng.uniform(-1., 1., (40, 2))))*observed_extent/2.
                world = local @ axes.T + [18., -3.]
                angle = math.radians(ego_heading)
                rotation = np.array([[math.cos(angle), -math.sin(angle)],
                                     [math.sin(angle), math.cos(angle)]])
                ego = dict(x=3., y=2., heading_deg=ego_heading)
                points = (world-[ego['x'], ego['y']]) @ rotation
                source = obstacle(x=18., y=-3., size=source_size, heading=object_heading)
                checked, = self.normalize([source], points=points, ego=ego)
                expected = physical+2.*(ma.ROAD_STATIC_MARGIN_M-ma.PORTABLE_STATIC_MARGIN_M)
                np.testing.assert_allclose(checked[6:8], expected, atol=1e-12)
                relative = (world-np.array(checked[:2])) @ axes
                self.assertTrue((np.abs(relative) <= np.array(checked[6:8])/2.-.05+1e-10).all())


if __name__ == '__main__':
    unittest.main()
