"""Tunnel protocol, source-time alignment and complete vehicle-body checks."""
import json
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from tunnel_no_go import (TunnelNoGoPublisher, TunnelNoGoReceiver, make_record,
                          TOPIC, FRAME)


class Clock:
    def __init__(self, value):
        self.value = value

    def __call__(self):
        return self.value


class TunnelNoGoTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock(50.)
        self.source_clock = Clock(100.)
        self.pose = dict(x=10., y=20., heading_deg=0., stamp=100.)
        self.walls = np.asarray([[[-20., 5.], [40., 5.]], [[-20., -5.], [40., -5.]]])
        self.path = np.column_stack((np.arange(0., 30.1, .5), np.zeros(61)))
        self.receiver = TunnelNoGoReceiver(topic=None, clock=self.clock, source_clock=self.source_clock)

    def snapshot(self, points=(), walls=None, pose=None, stamp=100., sequence=1):
        record = make_record(points, self.walls if walls is None else walls,
                             self.pose if pose is None else pose, stamp, True, sequence=sequence)
        self.assertTrue(self.receiver.ingest(json.dumps(record)))
        return self.receiver.snapshot(self.pose)

    def test_wire_roundtrip_clear_path(self):
        snapshot = self.snapshot()
        self.assertTrue(snapshot.valid)
        self.assertTrue(snapshot.active)
        self.assertEqual(snapshot['source_stamp'], 100.)
        self.assertEqual(snapshot.get('missing', 3), 3)
        self.assertEqual(snapshot.clear_distance(self.path), 30.)
        self.assertTrue(snapshot.path_clear(self.path))

    def test_front_footprint_blocks_before_path_centre(self):
        # The pose is the rear axle: the front bumper is 3.845 m ahead of each path
        # sample (+0.25 m margin, +0.125 m half-sample reserve), not 2.3175 m.
        snapshot = self.snapshot([[10., .2]])
        clear = snapshot.clear_distance(self.path)
        self.assertGreater(clear, 5.5)
        self.assertLessEqual(clear, 6.)   # first 0.25 m sample past 10 - 4.22 = 5.78 m
        self.assertFalse(snapshot.path_clear(self.path))

    def test_boolean_check_cannot_authorize_zero_invalid_or_tiny_blocked_horizon(self):
        snapshot = self.snapshot()
        for horizon in (0., -1., float('nan'), float('inf'), -float('inf')):
            with self.subTest(horizon=horizon):
                self.assertFalse(snapshot.path_clear(self.path, horizon))
        self.clock.value += .3
        stale = self.receiver.snapshot(self.pose)
        self.assertFalse(stale.path_clear(self.path, 0.))
        self.assertFalse(stale.path_clear(self.path, 30.))
        self.clock.value -= .3
        blocked = self.snapshot([[0., 0.]], sequence=2)
        self.assertFalse(blocked.path_clear(self.path, 1e-8))

    def test_rear_and_side_footprint_not_point_centre_only(self):
        # Rear bumper 0.79 m behind the rear-axle pose (+ margins), sides +-0.946 m.
        for point in ([[-1., 0.]], [[0., 1.1]], [[1., -.9]], [[3.5, 1.1]]):
            with self.subTest(point=point):
                self.receiver = TunnelNoGoReceiver(topic=None, clock=self.clock, source_clock=self.source_clock)
                self.assertEqual(self.snapshot(point).clear_distance(self.path), 0.)
        # 2 m behind the rear axle is 1.2 m behind the real rear bumper: no contact.
        self.receiver = TunnelNoGoReceiver(topic=None, clock=self.clock, source_clock=self.source_clock)
        self.assertEqual(self.snapshot([[-2., 0.]]).clear_distance(self.path), 30.)

    def test_wall_crossing_between_endpoints_blocks_full_box(self):
        walls = np.asarray([[[8., -6.], [8., 6.]], [[-20., -10.], [40., -10.]]])
        snapshot = self.snapshot(walls=walls)
        # Front bumper (3.845 m) + 0.25 m margin + 0.125 m reserve reaches x = 8 at 3.78 m.
        self.assertGreater(snapshot.clear_distance(self.path), 3.5)
        self.assertLessEqual(snapshot.clear_distance(self.path), 4.)

    def test_parallel_wall_blocks_only_when_body_reaches_it(self):
        path = self.path.copy()
        path[:, 1] = 4.
        self.assertEqual(self.snapshot().clear_distance(path), 0.)

    def test_finite_exit_has_no_closed_end_cap_or_wall_extension(self):
        walls = np.asarray([[[-20., 3.], [5., 3.]], [[-20., -3.], [5., -3.]]])
        # Remain centred until the rear body clears the physical wall end,
        # then curve into the road beyond it. A closed polygon or an infinite
        # wall would veto this physically open exit.
        x = np.arange(0., 30.1, .25)
        y = np.where(x <= 10., 0., np.minimum(6., .04*(x-10.)**2))
        path = np.column_stack((x, y))
        snapshot = self.snapshot(walls=walls)
        self.assertTrue(snapshot.path_clear(path), snapshot.clear_distance(path))

    def test_body_wall_endpoint_contact_at_exit_still_blocks(self):
        walls = np.asarray([[[-20., 3.], [5., 3.]], [[-20., -3.], [5., -3.]]])
        path = self.path.copy()
        path[:, 1] = 2.
        self.assertEqual(self.snapshot(walls=walls).clear_distance(path), 0.)

    def test_curved_path_checks_outside_corner(self):
        theta = np.linspace(0., np.pi/2, 81)
        path = np.column_stack((10.*np.sin(theta), 10.*(1.-np.cos(theta))))
        point = path[35]+np.asarray([1.1, -1.1])
        snapshot = self.snapshot([point], walls=np.asarray([[[-30., -30.], [40., -30.]]]))
        self.assertLess(snapshot.clear_distance(path), 10.)

    def test_source_timestamp_cannot_be_refreshed_by_republication(self):
        self.snapshot()
        self.clock.value += .28
        self.source_clock.value += .28
        self.assertTrue(self.receiver.ingest(make_record([], self.walls, self.pose, 100., True, sequence=2)))
        snapshot = self.receiver.snapshot(dict(self.pose, stamp=100.28))
        self.assertFalse(snapshot.valid)
        self.assertIn('source stale', snapshot.reason)
        self.assertEqual(snapshot.clear_distance(self.path), 0.)

    def test_receipt_disconnect_is_closed(self):
        self.snapshot()
        self.clock.value += .251
        snapshot = self.receiver.snapshot(self.pose)
        self.assertFalse(snapshot.valid)
        self.assertIn('receipt stale', snapshot.reason)

    def test_future_scan_is_closed(self):
        record = make_record([], self.walls, dict(self.pose, stamp=100.1), 100.1, True)
        self.assertTrue(self.receiver.ingest(record))
        snapshot = self.receiver.snapshot(self.pose)
        self.assertFalse(snapshot.valid)
        self.assertIn('future', snapshot.reason)

    def test_scan_geometry_pose_and_current_pose_are_separately_checked(self):
        self.snapshot()
        self.source_clock.value = 100.1
        self.assertFalse(self.receiver.snapshot(dict(self.pose, stamp=99.8)).valid)
        self.assertFalse(self.receiver.snapshot(dict(self.pose, stamp=100.2)).valid)
        with self.assertRaises(ValueError):
            make_record([], self.walls, dict(self.pose, stamp=100.3), 100., True)

    def test_coordinate_rotation_and_translation_same_physical_geometry(self):
        source_pose = dict(self.pose, x=0., y=0., heading_deg=90., speed_kmh=36.)
        record = make_record([[10., 0.]], self.walls, source_pose, 100., True)
        self.assertTrue(self.receiver.ingest(record))
        self.source_clock.value = 100.2
        target_pose = dict(self.pose, x=0., y=2., heading_deg=90., stamp=100.2, speed_kmh=36.)
        snapshot = self.receiver.snapshot(target_pose)
        self.assertTrue(snapshot.valid)
        self.assertTrue(np.allclose(snapshot.occupied_points, [[8., 0.]]))
        self.assertTrue(np.allclose(snapshot.walls[0], [[-22., 5.], [38., 5.]]))

    def test_aligned_pose_is_not_corrected_a_second_time(self):
        source_pose = dict(self.pose, x=10.7, y=20.4, heading_deg=2.)
        record = make_record([[15., -2.]], self.walls, source_pose, 100., True)
        self.assertTrue(self.receiver.ingest(record))
        snapshot = self.receiver.snapshot(source_pose)
        self.assertTrue(np.allclose(snapshot.occupied_points, [[15., -2.]]))
        self.assertTrue(np.allclose(snapshot.walls, self.walls))

    def test_fresh_old_geometry_cannot_authorize_after_teleport(self):
        self.snapshot()
        snapshot = self.receiver.snapshot(dict(self.pose, x=110.))
        self.assertFalse(snapshot.valid)
        self.assertIn('pose jump', snapshot.reason)
        self.assertEqual(snapshot.clear_distance(self.path), 0.)

    def test_measured_fast_motion_allowed_and_yaw_wrap_preserved(self):
        self.source_clock.value = 100.2
        source_pose = dict(self.pose, heading_deg=359., speed_kmh=28.94)
        self.assertTrue(self.receiver.ingest(make_record([], self.walls, source_pose, 100., True)))
        current = dict(self.pose, x=11.6, heading_deg=1., speed_kmh=28.94, stamp=100.2)
        self.assertTrue(self.receiver.snapshot(current).valid)

    def test_missing_map_frame_or_wrong_vehicle_frame_invalidates(self):
        for key, value in (('frame', 'lidar'), ('map_frame', 'odom'), ('schema', 2)):
            with self.subTest(key=key):
                record = make_record([], self.walls, self.pose, 100., True)
                record[key] = value
                self.assertFalse(self.receiver.ingest(record))
                self.assertFalse(self.receiver.snapshot(self.pose).valid)
        self.snapshot(sequence=4)
        self.assertFalse(self.receiver.snapshot(dict(self.pose, frame_id='odom')).valid)

    def test_malformed_message_discards_preceding_good_snapshot(self):
        self.snapshot()
        self.assertFalse(self.receiver.ingest('{broken'))
        self.assertFalse(self.receiver.snapshot(self.pose).valid)

    def test_invalid_geometry_shape_nan_missing_walls_and_zero_length(self):
        for points, walls in (([[1., float('nan')]], self.walls),
                              ([[1., 2., 3.]], self.walls), ([], []),
                              ([], [[[1., 1.], [1., 1.]]])):
            with self.subTest(points=points, walls=walls):
                with self.assertRaises(ValueError):
                    make_record(points, walls, self.pose, 100., True)

    def test_out_of_order_sequence_and_time_fail_closed(self):
        self.snapshot(sequence=2)
        self.assertFalse(self.receiver.ingest(make_record([], self.walls, self.pose, 100., True, sequence=1)))
        self.assertFalse(self.receiver.snapshot(self.pose).valid)
        self.assertTrue(self.receiver.ingest(make_record([], self.walls, self.pose, 100., True, sequence=3)))
        self.assertFalse(self.receiver.ingest(make_record([], self.walls, self.pose, 99.99, True, sequence=4)))

    def test_leave_tunnel_clears_previous_occupied_constraints(self):
        publisher = TunnelNoGoPublisher(topic=None)
        self.assertTrue(self.receiver.ingest(publisher.update([[10., 0.]], self.walls, self.pose, 100., True)))
        record = publisher.update([], [], None, 0., False)
        self.assertFalse(record['active'])
        self.assertTrue(self.receiver.ingest(record))
        snapshot = self.receiver.snapshot(self.pose)
        self.assertFalse(snapshot.active)
        self.assertFalse(snapshot.valid)
        self.assertEqual(snapshot.clear_distance(self.path), 0.)

    def test_publisher_failure_emits_invalid_instead_of_old_good_geometry(self):
        publisher = TunnelNoGoPublisher(topic=None)
        self.assertTrue(self.receiver.ingest(publisher.update([], self.walls, self.pose, 100., True)))
        record = publisher.update([[float('nan'), 0.]], self.walls, self.pose, 100., True)
        self.assertFalse(record['valid'])
        self.assertTrue(self.receiver.ingest(record))
        self.assertFalse(self.receiver.snapshot(self.pose).valid)

    def test_snapshot_copies_are_immutable(self):
        points = [[10., 0.]]
        record = make_record(points, self.walls, self.pose, 100., True)
        self.assertTrue(self.receiver.ingest(record))
        record['occupied_points'][0][0] = 0.
        points[0][0] = 0.
        snapshot = self.receiver.snapshot(self.pose)
        self.assertEqual(snapshot.occupied_points[0, 0], 10.)
        with self.assertRaises(ValueError):
            snapshot.occupied_points[0, 0] = 3.

    def test_duplicate_dense_path_sampling_and_invalid_paths(self):
        snapshot = self.snapshot()
        dense = np.column_stack((np.arange(0., 30.01, .01), np.zeros(3001)))
        duplicate = np.repeat(self.path, 2, axis=0)
        self.assertEqual(snapshot.clear_distance(dense), 30.)
        self.assertEqual(snapshot.clear_distance(duplicate), 30.)
        for path in ([], [[0., 0.]], [[0., 0.], [0., 0.]], [[0., 0.], [float('nan'), 1.]]):
            self.assertEqual(snapshot.clear_distance(path), 0.)
            self.assertFalse(snapshot.path_clear(path))

    def test_ros_wire_subscription_and_path_check_integration(self):
        callbacks, messages = {}, []

        class String:
            def __init__(self, data):
                self.data = data

        def subscribe(topic, message_type, callback, **kwargs):
            callbacks[topic] = callback
            return SimpleNamespace(unregister=lambda: None)

        def publisher(topic, message_type, **kwargs):
            def publish(message):
                messages.append(message.data)
                callbacks[topic](message)
            return SimpleNamespace(publish=publish, unregister=lambda: None)

        modules = {'rospy': SimpleNamespace(Subscriber=subscribe, Publisher=publisher,
                                           get_time=self.source_clock),
                   'std_msgs': SimpleNamespace(), 'std_msgs.msg': SimpleNamespace(String=String)}
        with patch.dict(sys.modules, modules):
            receiver = TunnelNoGoReceiver(clock=self.clock)
            publisher = TunnelNoGoPublisher()
            publisher.update([[10., 0.]], self.walls, self.pose, 100., True)
            snapshot = receiver.snapshot(self.pose)
            self.assertEqual(json.loads(messages[-1])['frame'], FRAME)
            self.assertIn(TOPIC, callbacks)
            self.assertTrue(snapshot.valid)
            self.assertLess(snapshot.clear_distance(self.path), 8.)
            self.assertFalse(snapshot.path_clear(self.path))
            publisher.update([], [], None, 0., False)
            self.assertFalse(receiver.snapshot(self.pose).active)
            receiver.close()
            publisher.close()


if __name__ == '__main__':
    unittest.main()
