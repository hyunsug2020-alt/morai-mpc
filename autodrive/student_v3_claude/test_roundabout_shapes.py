"""Observed point containment and publication-only roundabout shape changes."""
from dataclasses import replace
import json
import math
from pathlib import Path
import unittest

import numpy as np

from perception.roundabout_shapes import (RoundaboutShapeRegion, enclosing_publish_geometry,
                                         publication_detection, route_barrier_points, STOP_STATION_M)
from semantic_fusion import PedestrianPartMerger, Tracker
from roundabout_empty import MAX_ENTRY_SOURCE_AGE_S


HERE = Path(__file__).resolve().parent


class RoadsideReplayTests(unittest.TestCase):
    """Stopped scene: fresh scan, saved masks, ego displacement below 0.002 m."""
    @classmethod
    def setUpClass(cls):
        from gui import HdMapLanes
        with np.load(HERE/'test_data/roundabout_guardrail_20261013.npz') as data:
            cls.member, cls.source, cls.path, cls.raw = [data[k].copy()
                for k in ('member', 'ego', 'path', 'xyz')]
        cls.route = np.array([[p['x'], p['y']] for p in
            json.loads((HERE/'global_path.json').read_text())['waypoints']])
        cls.lanes = HdMapLanes(HERE/'link_set.json')
        cls.lanes.set_route(cls.route)
        cls.direction = np.arctan2(np.gradient(cls.route[:, 1]), np.gradient(cls.route[:, 0]))

    def source_geometry(self):
        xy, source = self.member[:, :2].astype(float), self.source
        yaw = source[3]
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw), math.cos(yaw)]])
        world = xy@rotation.T+source[1:3]
        lo, hi = xy.min(axis=0), xy.max(axis=0)
        local = (lo+hi)/2.
        original = dict(pos=local@rotation.T+source[1:3], local=local,
            size=np.maximum(hi-lo, .8)+.3, yaw=yaw, kind='obstacle',
            uid=37, vel=np.zeros(2), moving=False, n=len(xy), height=1.57)
        roadside = route_barrier_points(world, self.route, self.lanes.route_tree,
            self.direction, self.lanes.route_station, min_span=3.)
        retained = xy[~roadside]
        proposal = enclosing_publish_geometry(retained, source, original,
            roadside_points_excluded=int(roadside.sum()))
        return original, proposal, roadside, retained

    def test_partial_rail_refines_publication_without_changing_source_tracking(self):
        from roundabout_empty import semantic_entry_clearance, braking_reserve
        original, proposal, roadside, retained = self.source_geometry()
        self.assertIsNotNone(proposal)
        self.assertGreater(int(roadside.sum()), 400)
        self.assertGreaterEqual(len(retained), 4)
        source_position, source_size = original['pos'].copy(), original['size'].copy()
        original['publish_geometry'] = proposal
        output = publication_detection(original)
        np.testing.assert_array_equal(original['pos'], source_position)
        np.testing.assert_array_equal(original['size'], source_size)
        self.assertEqual(output['n'], len(self.member))
        self.assertEqual(output['roadside_points_excluded'], int(roadside.sum()))
        self.assertGreaterEqual(min(output['size']), 1.1)
        row = lambda item: (*item['pos'], max(item['size']), 'fusion_obstacle', 'obstacle',
            0., *item['size'], math.degrees(item['yaw']), 0., 0., 37)
        ego = dict(x=self.source[1], y=self.source[2], heading_deg=math.degrees(self.source[3]))
        self.assertLess(semantic_entry_clearance(self.path, ego, [row(original)]), braking_reserve(0., .2))
        self.assertGreater(semantic_entry_clearance(self.path, ego, [row(output)]), braking_reserve(3./3.6, .2))
        # Every unfiltered physical return still reaches the raw stopping proof.
        from roundabout_empty import raw_entry_clearance
        rotation = np.array([[math.cos(self.source[3]), -math.sin(self.source[3])],
                             [math.sin(self.source[3]), math.cos(self.source[3])]])
        world_path = self.path@rotation.T+self.source[1:3]
        for kmh in (0., 2.5, 3.):
            current = dict(ego, speed_kmh=kmh)
            raw_clear = raw_entry_clearance(self.lanes, current, self.raw[:, :2], world_path)
            if kmh == 0.:
                self.assertEqual(raw_clear, braking_reserve(0., MAX_ENTRY_SOURCE_AGE_S))
            else:
                # This stopped frame is already past the line. Its longer
                # turning prefix does not certify another 2.5/3 km/h empty
                # roll; refining the box must not bypass that raw rejection.
                self.assertEqual(raw_clear, 0.)
            real_blocker = np.vstack((self.raw[:, :2], [[1., 0.]]))
            self.assertEqual(raw_entry_clearance(self.lanes, current, real_blocker, world_path), 0.)

    def test_normal_road_span_and_moving_or_person_bodies_remain_unchanged(self):
        original, proposal, _, _ = self.source_geometry()
        world = proposal.world_points
        ordinary = route_barrier_points(world, self.route, self.lanes.route_tree,
            self.direction, self.lanes.route_station)
        self.assertFalse(ordinary.any())
        for changes in (dict(kind='pedestrian'), dict(kind='npc'),
                        dict(vel=np.array([.001, 0.])), dict(merged_part_ids=[3])):
            detection = dict(original, publish_geometry=proposal, **changes)
            self.assertIs(publication_detection(detection), detection)


class RoundaboutShapeTests(unittest.TestCase):
    @staticmethod
    def points(length=3., width=1., angle=math.radians(5.)):
        corners = np.array([[-1., -1.], [-1., 1.], [1., -1.], [1., 1.]])
        points = corners*[length/2., width/2.]
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        return np.repeat(points @ rotation.T + [6., 2.], 2, axis=0)

    @staticmethod
    def source_ego(yaw=.7):
        return (10., -99.4, 312., yaw, 0., 0.)

    def original(self, points, ego):
        lo, hi = points.min(axis=0), points.max(axis=0)
        local = (lo+hi)/2.
        rotation = np.array([[math.cos(ego[3]), -math.sin(ego[3])],
                             [math.sin(ego[3]), math.cos(ego[3])]])
        return dict(pos=local @ rotation.T + ego[1:3], local=local,
                    size=np.maximum(hi-lo, .8)+.3, yaw=ego[3], height=1.5,
                    n=len(points), motion_bounds=np.array([lo, hi]),
                    kind='obstacle', uid=40, vel=np.zeros(2), moving=False)

    def proposal(self, points=None, ego=None):
        points = self.points() if points is None else points
        ego = self.source_ego() if ego is None else ego
        original = self.original(points, ego)
        candidate = enclosing_publish_geometry(points, ego, original)
        self.assertIsNotNone(candidate)
        return points, ego, original, candidate

    def assert_source_points_preserved(self, points, ego, candidate):
        yaw = candidate.yaw
        axis = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
        source_axis = np.array([[math.cos(ego[3]), -math.sin(ego[3])],
                                [math.sin(ego[3]), math.cos(ego[3])]])
        world = points @ source_axis.T + ego[1:3]
        # Independent per-face projection oracle includes .15 m padding.
        relative = (world-candidate.position) @ axis
        available = np.asarray(candidate.size)/2.-np.abs(relative)
        self.assertGreaterEqual(float(available.min()), .15-1e-9)

    def test_old_area_threshold_rejected_a_valid_tighter_rectangle(self):
        points, ego, original, candidate = self.proposal()
        raw_area = np.prod(points.max(axis=0)-points.min(axis=0))
        self.assertGreater(3./raw_area, .75)
        self.assertLess(np.prod(candidate.size), np.prod(original['size']))
        self.assertAlmostEqual(np.prod(candidate.size), 3.3*1.3, places=7)
        self.assert_source_points_preserved(points, ego, candidate)

    def test_world_rotation_translation_and_all_member_points_are_preserved(self):
        points = np.vstack((self.points(6., 1.7, .43), [[6.1, 2.2], [5.7, 1.9]]))
        for yaw in (-2.1, 0., 1.56):
            with self.subTest(yaw=yaw):
                points, ego, original, candidate = self.proposal(points, self.source_ego(yaw))
                self.assert_source_points_preserved(points, ego, candidate)
                self.assertGreaterEqual(min(candidate.size), 1.1-1e-12)

    def test_degenerate_thin_members_keep_the_original_minimum_dimension(self):
        line = np.column_stack((np.linspace(-2., 2., 9), np.zeros(9)))
        angle = .4
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        points, ego, original, candidate = self.proposal(line @ rotation.T + [6., 2.])
        self.assertAlmostEqual(min(candidate.size), 1.1)
        self.assert_source_points_preserved(points, ego, candidate)

    def test_no_area_gain_preserves_original_geometry(self):
        points = self.points(angle=0.)
        ego = self.source_ego()
        original = self.original(points, ego)
        self.assertIsNone(enclosing_publish_geometry(points, ego, original))

    def test_nonfinite_missing_or_malformed_members_use_the_original_box(self):
        points = self.points(); ego = self.source_ego(); original = self.original(points, ego)
        bad = points.copy(); bad[0, 0] = math.nan
        for member in (bad, points[:3], np.zeros((5, 3)), [], None):
            with self.subTest(member=repr(member)):
                self.assertIsNone(enclosing_publish_geometry(member, ego, original))
        for changed in ({'size': [math.inf, 1.1]}, {'pos': [math.nan, 0.]},
                        {'yaw': math.nan}, {'size': [.5, 1.1]}):
            self.assertIsNone(enclosing_publish_geometry(points, ego, dict(original, **changed)))
        self.assertIsNone(enclosing_publish_geometry(points, (*ego[:3], math.nan, 0., 0.), original))

    def test_publication_changes_only_stationary_obstacle_geometry(self):
        points, ego, original, candidate = self.proposal()
        original['publish_geometry'] = candidate
        saved = {key: np.asarray(original[key]).copy() for key in
                 ('pos', 'local', 'size', 'motion_bounds', 'vel')}
        output = publication_detection(original)
        self.assertIsNot(output, original)
        for key, value in saved.items():
            np.testing.assert_array_equal(original[key], value)
        for key in ('uid', 'kind', 'height', 'n', 'moving'):
            self.assertEqual(output[key], original[key])
        self.assertIs(output['motion_bounds'], original['motion_bounds'])
        self.assertIs(output['vel'], original['vel'])
        self.assert_source_points_preserved(points, ego, candidate)

    def test_people_npcs_parked_sources_and_any_nonzero_velocity_are_unchanged(self):
        _, _, original, candidate = self.proposal()
        original['publish_geometry'] = candidate
        for changes in ({'kind': 'pedestrian'}, {'kind': 'npc'},
                        {'kind': 'npc', 'moving': False}, {'merged_part_ids': [41]},
                        {'vel': np.array([1e-12, 0.])}, {'vel': np.array([.6, 0.])},
                        {'vel': np.array([math.nan, 0.])}):
            with self.subTest(changes=changes):
                row = dict(original, **changes)
                self.assertIs(publication_detection(row), row)

    def test_damaged_or_mismatched_candidate_falls_back_without_mutation(self):
        _, _, original, candidate = self.proposal()
        for changed in (replace(candidate, size=(1.1, 1.1)),
                        replace(candidate, yaw=math.nan),
                        replace(candidate, position=(0., 0.)), {'size': (1., 1.)}):
            row = dict(original, publish_geometry=changed)
            self.assertIs(publication_detection(row), row)
        row = dict(original, pos=original['pos']+[.001, 0.], publish_geometry=candidate)
        self.assertIs(publication_detection(row), row)

    def test_member_mutation_cannot_change_a_candidate_snapshot(self):
        points, ego, _, candidate = self.proposal()
        saved = candidate.world_points.copy()
        points[:] = 999.
        np.testing.assert_array_equal(candidate.world_points, saved)
        with self.assertRaises(ValueError):
            candidate.world_points[0, 0] = 999.

    def test_tracker_velocity_inputs_and_outputs_keep_original_geometry(self):
        points, ego, original, candidate = self.proposal()
        original['publish_geometry'] = candidate
        tracker = Tracker()
        row = tracker.update('obstacle', [original], 10.)[0]
        for key in ('pos', 'size', 'motion_bounds'):
            np.testing.assert_array_equal(row[key], original[key])
        self.assertEqual(row['yaw'], original['yaw'])
        self.assertIs(row['publish_geometry'], candidate)
        np.testing.assert_array_equal(tracker.tracks[row['uid']]['pos'], original['pos'])

    def test_pedestrian_part_merger_uses_original_inputs_before_publication(self):
        _, _, original, candidate = self.proposal()
        original = dict(original, uid=2, pos=np.array([5.1, .1]), size=np.array([1.1, 1.1]),
                        vel=np.array([1., 0.]), publish_geometry=candidate)
        pedestrian = dict(uid=1, kind='pedestrian', pos=np.array([5., 0.]),
                          size=np.array([.6, .6]), vel=np.array([1., 0.]), height=1.6,
                          n=10, moving=True)
        tagged, plain = PedestrianPartMerger(), PedestrianPartMerger()
        without = {key: value for key, value in original.items() if key != 'publish_geometry'}
        for stamp in (10., 10.08, 10.16):
            actual = tagged.merge([pedestrian, original], stamp)
            expected = plain.merge([pedestrian, without], stamp)
            self.assertEqual([o['uid'] for o in actual], [o['uid'] for o in expected])
            for a, b in zip(actual, expected):
                for key in ('pos', 'size', 'vel'):
                    np.testing.assert_array_equal(a[key], b[key])
                self.assertIs(publication_detection(a), a)


class RoundaboutRegionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.route = np.array([[p['x'], p['y']] for p in
            json.loads((HERE/'global_path.json').read_text())['waypoints']])
        cls.arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(cls.route, axis=0), axis=1))]

    def pose(self, station):
        xy = [np.interp(station, self.arc, self.route[:, i]) for i in (0, 1)]
        return (10., *xy, math.pi/2., 0., 0.)

    def test_only_known_route_and_source_station_window_are_admitted(self):
        region = RoundaboutShapeRegion(self.route)
        self.assertTrue(region.certified)
        for station, expected in ((STOP_STATION_M-12.9, True), (STOP_STATION_M+13.9, True),
                                  (STOP_STATION_M-13.1, False), (STOP_STATION_M+14.1, False),
                                  (0., False)):
            with self.subTest(station=station):
                self.assertEqual(region.active(self.pose(station)), expected)

    def test_unknown_changed_nonfinite_or_malformed_route_is_disabled(self):
        changed = self.route.copy(); changed[100, 0] += 1e-6
        for route in (changed, self.route[:, :1], [], np.array([[math.nan, 0.], [1., 0.]])):
            self.assertFalse(RoundaboutShapeRegion(route).active(self.pose(STOP_STATION_M)))

    def test_invalid_source_pose_is_disabled(self):
        region = RoundaboutShapeRegion(self.route)
        for pose in (None, (10., 0.), (math.nan, -99.4, 312., 1.5, 0., 0.),
                     (0., -99.4, 312., 1.5, 0., 0.),
                     (10., -99.4, 312., 1.5, math.inf, 0.)):
            self.assertFalse(region.active(pose))

    def test_route_mutation_does_not_move_the_certified_source_region(self):
        route = self.route.copy(); region = RoundaboutShapeRegion(route)
        pose = self.pose(STOP_STATION_M)
        route[:] = 999.
        self.assertTrue(region.active(pose))


if __name__ == '__main__':
    unittest.main()
