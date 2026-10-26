"""Replay the observed rail/cushion without erasing physical collision inputs."""
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from perception.roadside_structures import RoadsideStructures, roadside_structure


ROOT = Path(__file__).resolve().parent


def world_points(points, pose):
    c, s = math.cos(pose[3]), math.sin(pose[3])
    return points[:, :2] @ np.array([[c, s], [-s, c]])+pose[1:3]


class RoadsideStructureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.route = np.asarray([[p['x'], p['y']] for p in
            json.loads((ROOT/'global_path.json').read_text())['waypoints']])
        cls.survey = RoadsideStructures(cls.route)
        with np.load(ROOT/'test_data/roundabout_roadside_20261013.npz') as data:
            cls.replay = {key: data[key].copy() for key in data.files}
        with np.load(ROOT/'test_data/roundabout_roadside_close_20261013.npz') as data:
            cls.close = {key: data[key].copy() for key in data.files}
        with np.load(ROOT/'test_data/roundabout_guardrail_20261013.npz') as data:
            cls.previous = {key: data[key].copy() for key in data.files}

    def source_detection(self, member=None, replay=None):
        replay = self.replay if replay is None else replay
        member = replay['member'] if member is None else member
        ego = replay['ego']
        lo, hi = member[:, :2].min(axis=0), member[:, :2].max(axis=0)
        center = (lo+hi)/2.
        detection = dict(kind='obstacle', pos=world_points(center[None, :], ego)[0],
            local=center, size=np.maximum(hi-lo, .8)+.3, yaw=ego[3],
            height=1.55, n=len(member), vel=np.zeros(2), moving=False, uid=51)
        evidence = self.survey.semantic_evidence(detection, world_points(member, ego), ego)
        if evidence is not None:
            detection['roadside_evidence'] = evidence
        return detection

    def test_both_stationary_views_and_short_cropped_clusters_match(self):
        self.assertTrue(self.survey.certified)
        for replay in (self.previous, self.replay, self.close):
            points = world_points(replay['member'], replay['ego'])
            self.assertIsNotNone(self.survey.association(points, replay['ego']))
        for replay in (self.replay, self.close):
            groups = replay['groups']
            for group in np.unique(groups):
                member = replay['member'][groups == group]
                self.assertIsNotNone(roadside_structure(self.source_detection(member, replay)))

    def test_tracking_and_merging_preserve_the_source_evidence(self):
        from semantic_fusion import Tracker, PedestrianPartMerger
        detection = self.source_detection()
        detection.pop('uid')
        tracker = Tracker()
        rows = tracker.update('obstacle', [detection], self.replay['ego'][0])
        merged = PedestrianPartMerger().merge([dict(rows[0], kind='obstacle')], self.replay['ego'][0])
        self.assertIsNotNone(roadside_structure(merged[0]))

    def test_unknown_return_in_same_cluster_keeps_the_whole_obstacle(self):
        member = np.vstack((self.replay['member'], [[1., 0., .8]]))
        detection = self.source_detection(member)
        self.assertIsNone(roadside_structure(detection))
        self.assertEqual(detection['n'], len(member))

    def test_point_classification_separates_infrastructure_before_mixed_clustering(self):
        data = self.replay
        unknown = np.array([[1., 0., .8], [1.2, .1, .9], [1.4, -.1, 1.], [1.6, 0., .8]])
        mixed = np.vstack((data['member'], unknown))
        saved = mixed.copy()
        mapped = self.survey.local_point_mask(mixed, data['ego'])
        self.assertTrue(mapped[:len(data['member'])].all())
        self.assertFalse(mapped[len(data['member']):].any())
        np.testing.assert_array_equal(mixed[~mapped], unknown)
        np.testing.assert_array_equal(mixed, saved)

    def test_vehicle_input_filter_keeps_new_on_road_car_points_and_raw_cloud(self):
        data = self.replay
        car = np.array([[6., 0., 1.], [6.5, .5, 1.2], [7., -.5, 1.3]])-data['mount']
        original = np.vstack((data['points'], car)); saved = original.copy()
        mapped = self.survey.local_point_mask(original, data['ego'][:4], mount=data['mount'])
        self.assertGreater(int(mapped.sum()), 400)
        self.assertFalse(mapped[-len(car):].any())
        np.testing.assert_array_equal(original, saved)
        np.testing.assert_array_equal(original[~mapped][-len(car):], car)

    def test_mapped_section_does_not_turn_the_remaining_rail_into_short_boxes(self):
        from scipy.spatial import cKDTree
        from semantic_fusion import filter_infrastructure_points
        from perception.roundabout_shapes import route_barrier_points
        route = self.route
        tree = cKDTree(route)
        station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        direction = np.arctan2(np.gradient(route[:, 1]), np.gradient(route[:, 0]))
        def barrier(points):
            return route_barrier_points(points, route, tree, direction, station)
        world = np.column_stack((np.full(100, -101.6), np.linspace(308., 317., 100)))
        pose = self.replay['ego']
        c, s = math.cos(pose[3]), math.sin(pose[3])
        local = (world-pose[1:3]) @ np.array([[c, -s], [s, c]])
        rail = np.column_stack((local, np.ones(len(local))))
        mapped = self.survey.local_point_mask(rail, pose)
        self.assertGreater(int(mapped.sum()), 40)
        self.assertFalse(barrier(world[~mapped]).any())
        self.assertTrue(barrier(world).all())
        car_world = np.array([[-99., 311.], [-99.1, 311.2], [-99., 311.4], [-98.9, 311.6]])
        car_local = (car_world-pose[1:3]) @ np.array([[c, -s], [s, c]])
        car = np.column_stack((car_local, np.ones(len(car_local))))
        original = np.vstack((rail, car)); saved = original.copy()
        kept, line_count, _ = filter_infrastructure_points(original, 'obstacle', pose,
                                                          self.survey, barrier)
        self.assertEqual(line_count, len(rail))
        np.testing.assert_allclose(kept, car)
        np.testing.assert_array_equal(original, saved)
        people, line_count, mapped_count = filter_infrastructure_points(rail, 'pedestrian', pose,
                                                                       self.survey, barrier)
        np.testing.assert_array_equal(people, rail)
        self.assertEqual((line_count, mapped_count), (0, 0))

    def test_people_vehicles_and_merged_fragments_are_not_suppressed(self):
        source = self.source_detection()
        for changes in (dict(kind='npc'), dict(kind='pedestrian'), dict(vel=np.array([math.nan, 0.])),
                        dict(merged_part_ids=[53]), dict(pos=source['pos']+[.01, 0.]),
                        dict(size=source['size']+[.01, 0.]), dict(yaw=source['yaw']+.01)):
            with self.subTest(changes=changes):
                self.assertIsNone(roadside_structure(dict(source, **changes)))

    def test_registered_structure_identity_survives_centroid_velocity_noise(self):
        source = self.source_detection()
        self.assertIsNotNone(roadside_structure(dict(source, moving=True, vel=np.array([.82, 0.]))))
        unknown = self.source_detection(np.vstack((self.replay['member'], [[1., 0., .8]])))
        self.assertIsNone(roadside_structure(dict(unknown, moving=True, vel=np.array([.82, 0.]))))

    def test_aligned_moving_views_cover_the_same_fixed_infrastructure(self):
        with np.load(ROOT/'test_data/roundabout_roadside_sweep_20261013.npz') as data:
            for key in data.files:
                self.assertIsNotNone(self.survey.association(data[key], self.replay['ego']))

    def test_pointpillar_false_car_requires_all_actual_3d_support(self):
        for data in (self.replay, self.close):
            saved = data['points'].copy()
            result = self.survey.vehicle_structure(data['points'], data['box'], data['mount'], data['ego'][:4])
            self.assertIsNotNone(result)
            self.assertGreater(result[1], 400)
            np.testing.assert_array_equal(data['points'], saved)

    def test_real_parked_car_support_inside_the_model_box_remains_a_vehicle(self):
        data = self.replay
        # The false model box covers empty road next to the rail. A new return
        # there must invalidate association even when 499 rail returns match.
        road_return = np.array([1.5, 1.5, .8])-data['mount']
        added = np.vstack((data['points'], road_return))
        self.assertIsNone(self.survey.vehicle_structure(added, data['box'], data['mount'], data['ego'][:4]))
        real_box = data['box'].copy()
        real_box[:2] = np.array([5., 0.])-data['mount'][:2]
        self.assertIsNone(self.survey.vehicle_structure(data['points'], real_box, data['mount'], data['ego'][:4]))

    def test_sparse_or_invalid_model_support_cannot_exclude_a_car(self):
        data = self.replay
        for points in (data['points'][:3], np.empty((0, 3)), np.array([[math.nan, 0., 0.]])):
            self.assertIsNone(self.survey.vehicle_structure(points, data['box'], data['mount'], data['ego']))
        for box in (data['box'][:6], np.zeros(7), np.full(7, math.nan)):
            self.assertIsNone(self.survey.vehicle_structure(data['points'], box, data['mount'], data['ego']))

    def test_source_pose_and_approach_bounds_are_required(self):
        data = self.replay
        points = world_points(data['member'], data['ego'])
        for pose in (None, [1., 2.], [math.nan, *data['ego'][1:]],
                     [0., *data['ego'][1:]], [data['ego'][0], 0., 0., 0., 0., 0.]):
            self.assertIsNone(self.survey.association(points, pose))
        near = self.survey.region
        for station in (820., 880., 920., 700., 1000.):
            index = int(np.argmin(abs(near.stations-station)))
            x, y = near.starts[index]
            self.assertEqual(self.survey.active([10., x, y, 0.]), 805.1 <= near.stations[index] <= 939.84)

    def test_changed_route_map_or_survey_disables_association(self):
        route = self.route.copy(); route[0, 0] += .01
        self.assertFalse(RoadsideStructures(route).certified)
        payload = json.loads((ROOT/'roadside_structures.json').read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'survey.json'; map_path = Path(directory)/'map.json'
            map_path.write_text('{}')
            self.assertFalse(RoadsideStructures(self.route, map_path=map_path).certified)
            for changed in (dict(schema=2), dict(map_sha256='bad'), dict(association_tolerance_m=.251),
                            dict(station_window_m=[940., 805.]), dict(structures=[]),
                            dict(structures=[dict(id='bad', observed_footprint_xy=[[0., 0.]])])):
                path.write_text(json.dumps(dict(payload, **changed)))
                self.assertFalse(RoadsideStructures(self.route, survey_path=path).certified)
            path.write_text('{')
            self.assertFalse(RoadsideStructures(self.route, survey_path=path).certified)

    def test_raw_collision_check_keeps_registered_points_and_real_blockers(self):
        from gui import HdMapLanes
        from roundabout_empty import raw_entry_clearance
        lanes = HdMapLanes(ROOT/'link_set.json'); lanes.set_route(self.route)
        data = self.previous
        pose = dict(x=data['ego'][1], y=data['ego'][2],
            heading_deg=math.degrees(data['ego'][3]), speed_kmh=0.)
        path = world_points(data['path'], data['ego'])
        original = data['xyz'][:, :2].copy()
        self.assertGreater(raw_entry_clearance(lanes, pose, original, path), 0.)
        blocked = np.vstack((original, [[1., 0.]]))
        self.assertEqual(raw_entry_clearance(lanes, pose, blocked, path), 0.)
        np.testing.assert_array_equal(data['xyz'][:, :2], original)


if __name__ == '__main__':
    unittest.main()
