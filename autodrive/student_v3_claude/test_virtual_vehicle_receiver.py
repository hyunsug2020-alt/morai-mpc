"""Synthetic detector delivery retains the real receiver's atomic contract."""
import json
import threading
import unittest
from unittest.mock import patch

from junction_source_guard import current_frame, frame_identity
import virtual_sim as sim


class VirtualVehicleReceiverTests(unittest.TestCase):
    def row(self, uid=7, x=10., valid=True, prediction_age=.0):
        return (x, 2., 5., 'lidar_car', 'npc', 3., 5., 2.1, 0.,
                3., 0., uid, 0., None, valid, prediction_age)

    def deliver(self, receiver, rows, now=100., stamp=100., **kwargs):
        with patch('time.monotonic', return_value=now):
            receiver.publish_rows(rows, stamp=stamp, **kwargs)

    def test_snapshot_and_callback_are_the_production_methods(self):
        self.assertIs(sim.SimulatedVehicleReceiver.snapshot, sim.gui.VehicleReceiver.snapshot)
        self.assertIs(sim.SimulatedVehicleReceiver._on_message, sim.gui.VehicleReceiver._on_message)

    def test_full_frame_and_motion_flags_round_trip_through_real_snapshot(self):
        receiver = sim.SimulatedVehicleReceiver()
        self.deliver(receiver, [self.row(valid=False, prediction_age=.2)], unknown_ids=[7])
        with patch('time.monotonic', return_value=100.1):
            objects, ready, status = receiver.snapshot()
        self.assertTrue(ready)
        self.assertEqual(objects[0], self.row(valid=False, prediction_age=.2))
        self.assertEqual(status['vehicles'][0]['position'], [10., 2.])
        self.assertEqual(status['unknown_ids'], [7])
        self.assertEqual(status['age_s'], .1)
        self.assertEqual(receiver.received, 100.)

    def test_lost_delivery_does_not_renew_source_or_receipt(self):
        receiver = sim.SimulatedVehicleReceiver()
        self.deliver(receiver, [self.row()])
        original = receiver.record
        receiver.delivery_enabled = False
        self.deliver(receiver, [self.row(uid=8)], now=100.36, stamp=100.36)
        with patch('time.monotonic', return_value=100.36):
            objects, ready, status = receiver.snapshot()
        self.assertIs(receiver.record, original)
        self.assertEqual(receiver.received, 100.)
        self.assertFalse(ready)
        self.assertEqual(objects, [])
        self.assertEqual(status['age_s'], .36)

    def test_same_stamp_revision_changes_complete_frame_identity(self):
        receiver = sim.SimulatedVehicleReceiver()
        self.deliver(receiver, [self.row()])
        old_record = receiver.record
        expected = frame_identity(old_record)
        self.deliver(receiver, [self.row(x=9.)], now=100.1)
        self.assertIsNot(receiver.record, old_record)
        with patch('time.monotonic', return_value=100.1):
            self.assertEqual(current_frame(receiver, expected), (False, 'changed'))

    def test_invalid_callback_does_not_replace_a_complete_frame(self):
        receiver = sim.SimulatedVehicleReceiver()
        self.deliver(receiver, [self.row()])
        original = receiver.record
        with patch('time.monotonic', return_value=101.):
            receiver._on_message(sim.Message(json.dumps(dict(vehicles=None))))
        self.assertIs(receiver.record, original)
        self.assertEqual(receiver.received, 100.)

    def test_callback_cannot_replace_frame_while_publication_owns_receiver_lock(self):
        receiver = sim.SimulatedVehicleReceiver()
        self.deliver(receiver, [self.row()])
        original = receiver.record
        started, finished = threading.Event(), threading.Event()

        def callback():
            started.set()
            self.deliver(receiver, [self.row(uid=8)], now=100.1)
            finished.set()

        with receiver.lock:
            worker = threading.Thread(target=callback)
            worker.start()
            self.assertTrue(started.wait(1.))
            self.assertFalse(finished.wait(.01))
            self.assertIs(receiver.record, original)
        worker.join(1.)
        self.assertFalse(worker.is_alive())
        self.assertTrue(finished.is_set())
        self.assertEqual(receiver.record['vehicles'][0]['uid'], 8)


class VirtualFullProviderSourceRaceTests(unittest.TestCase):
    def runtime(self, *, gap=.55, speed_kmh=0.):
        from types import SimpleNamespace
        import numpy as np
        import test_live_path_safety as fixture
        from test_roundabout_empty_entry import LINE
        from test_roundabout_yield_entry import planner
        from stop_line import competition_stop_controller
        helper = fixture.LivePathSafetyTests()
        runtime = helper.attach(helper.runtime())
        runtime.gap_planner, ego = planner(gap, speed_kmh)
        runtime.test_status['ego'].update(ego)
        runtime.lanes = SimpleNamespace(route_progress=lambda e: e['x'])
        runtime.on_road_vehicles = lambda rows: list(rows)
        runtime.stop_line_controller = competition_stop_controller([LINE])
        runtime.stop_line_controller.update(ego['x'], speed_kmh/3.6, 100., True,
            observation_age_s=.01, empty_front_station=50.-gap)
        runtime.vehicle_receiver = sim.SimulatedVehicleReceiver()
        runtime.test_scan.update(stamp=100., age_s=.01,
                                obstacle_points=np.empty((0, 2)))
        runtime.object_receiver.observation_stamp = 100.
        runtime.path_publisher.zone_mps = 2.78
        runtime.path_publisher.emergency_stop = False
        runtime.path_publisher._safety_provider.owner = 'ROAD'
        return helper, runtime

    def checked_payload(self, helper, runtime):
        publisher = runtime.path_publisher
        if publisher.waypoints_pub.items:
            return helper.payload(runtime)
        # Invalid first geometry still publishes a real zero speed command;
        # there is no prior path to repeat. Inspect the actual rejected plan.
        return dict(target_speed_mps=float(publisher.velocity_pub.items[-1].data),
                    decision=publisher._active_decision_plan.summary())

    def publish(self, runtime, *, gap_checked=True):
        from test_roundabout_empty_entry import PATH
        runtime.path_publisher.publish(runtime.test_status['ego'], PATH, PATH,
            dict(mode='ROUTE BASE', reason='route', junction_body_cap_mps=8.,
                 stop_line=dict(gap_checked=gap_checked)))

    def test_repeated_new_frames_keep_a_checked_wait_braking_cap_without_retry(self):
        from test_junction_entry import car
        helper, runtime = self.runtime(gap=8., speed_kmh=21.6)
        receiver = runtime.vehicle_receiver
        provider = runtime.path_publisher._safety_provider
        original = provider.junction_follow
        calls = []
        ego = runtime.test_status['ego']

        def vehicle(stamp):
            # Same actor, consistent measured motion across complete frames.
            return car(ego['x']+15., 6.-5.5*(stamp-99.8), 0., -5.5, 2)

        def update_during_wait(*args, **kwargs):
            calls.append(kwargs.get('yield_source_frame'))
            stamp = 99.8+.05*len(calls)
            receiver.publish_rows([vehicle(stamp)], stamp=stamp)
            return original(*args, **kwargs)

        provider.junction_follow = update_during_wait
        with patch('time.monotonic', return_value=100.):
            receiver.publish_rows([vehicle(99.8)], stamp=99.8)
            for index in range(4):
                self.publish(runtime, gap_checked=False)
                payload = self.checked_payload(helper, runtime)
                validation = payload['decision']['validation']
                self.assertEqual(validation['live_safety']['junction_traffic']
                                 ['ring']['action'], 'WAIT')
                self.assertGreater(payload['target_speed_mps'], 0.)
                self.assertLess(payload['target_speed_mps'], ego['speed_kmh']/3.6)
                self.assertFalse(payload['decision']['emergency'])
                self.assertNotIn('junction_source_retry', validation)
                self.assertEqual(len(calls), index+1)
                self.assertFalse(runtime.gap_planner.junction_committed)
                self.assertFalse(runtime.stop_line_controller.released)

    def test_current_raw_contact_still_stops_a_wait_during_source_updates(self):
        import numpy as np
        from test_junction_entry import car
        helper, runtime = self.runtime(gap=8., speed_kmh=21.6)
        receiver = runtime.vehicle_receiver
        provider = runtime.path_publisher._safety_provider
        original = provider.junction_follow
        ego = runtime.test_status['ego']
        runtime.test_scan['obstacle_points'] = np.array([[3., 0.]])

        def update_during_wait(*args, **kwargs):
            receiver.publish_rows([car(ego['x']+15., 5.725, 0., -5.5, 2)],
                                  stamp=99.85)
            return original(*args, **kwargs)

        provider.junction_follow = update_during_wait
        with patch('time.monotonic', return_value=100.):
            receiver.publish_rows([car(ego['x']+15., 6., 0., -5.5, 2)], stamp=99.8)
            self.publish(runtime, gap_checked=False)
        payload = self.checked_payload(helper, runtime)
        self.assertEqual(payload['target_speed_mps'], 0.)
        self.assertTrue(payload['decision']['emergency'])
        self.assertFalse(runtime.gap_planner.junction_committed)
        self.assertFalse(runtime.stop_line_controller.released)

    def assert_wait_replaces_old_go(self, initial_stamp, *, same_stamp_revision=False):
        from test_junction_entry import car
        helper, runtime = self.runtime()
        receiver = runtime.vehicle_receiver
        provider = runtime.path_publisher._safety_provider
        original = provider.junction_follow
        calls = []

        def arriving_wait(*args, **kwargs):
            calls.append(kwargs.get('yield_source_frame'))
            if len(calls) == 1:
                ego = runtime.test_status['ego']
                receiver.publish_rows([car(ego['x']+10., 6., 0., -5.5, 2)], stamp=100.)
            return original(*args, **kwargs)

        provider.junction_follow = arriving_wait
        with patch('time.monotonic', return_value=100.):
            receiver.publish_rows([], stamp=initial_stamp)
            self.publish(runtime)
        payload = self.checked_payload(helper, runtime)
        self.assertEqual(len(calls), 1 if same_stamp_revision else 2)
        self.assertEqual(payload['target_speed_mps'], 0.)
        if same_stamp_revision:
            self.assertTrue(payload['decision']['validation']['junction_source_same_stamp_revision'])
        else:
            self.assertEqual(payload['decision']['validation']['live_safety']
                             ['junction_traffic']['ring']['action'], 'WAIT')
        self.assertIn('junction_source_retry', payload['decision']['validation'])
        self.assertFalse(runtime.gap_planner.junction_committed)
        self.assertFalse(runtime.stop_line_controller.released)

    def test_new_complete_wait_replaces_an_old_go_before_any_commit(self):
        self.assert_wait_replaces_old_go(99.99)

    def test_same_stamp_wait_revision_cannot_reuse_old_empty_geometry(self):
        self.assert_wait_replaces_old_go(100., same_stamp_revision=True)

    def test_lost_detector_delivery_is_stale_in_the_actual_full_provider(self):
        helper, runtime = self.runtime()
        receiver = runtime.vehicle_receiver
        with patch('time.monotonic', return_value=99.6):
            receiver.publish_rows([], stamp=99.6)
        receiver.delivery_enabled = False
        with patch('time.monotonic', return_value=100.):
            receiver.publish_rows([], stamp=100.)
            self.publish(runtime)
        payload = self.checked_payload(helper, runtime)
        self.assertEqual(receiver.received, 99.6)
        self.assertEqual(payload['target_speed_mps'], 0.)
        self.assertTrue(payload['decision']['emergency'])
        self.assertFalse(runtime.gap_planner.junction_committed)
        self.assertFalse(runtime.stop_line_controller.released)

    def test_a_second_source_change_fails_closed_without_a_third_attempt(self):
        from test_junction_entry import car
        helper, runtime = self.runtime()
        receiver = runtime.vehicle_receiver
        provider = runtime.path_publisher._safety_provider
        original = provider.junction_follow
        calls = []

        def changing_source(*args, **kwargs):
            calls.append(kwargs.get('yield_source_frame'))
            # Every replacement is independently clear. The source race alone
            # must reject the second attempt, with no third classification.
            receiver.publish_rows([car(1000.+len(calls), 0., 3., 0., len(calls))],
                                  stamp=99.98+.01*len(calls))
            return original(*args, **kwargs)

        provider.junction_follow = changing_source
        with patch('time.monotonic', return_value=100.):
            receiver.publish_rows([], stamp=99.98)
            self.publish(runtime)
        payload = self.checked_payload(helper, runtime)
        self.assertEqual(len(calls), 2)
        self.assertEqual(payload['target_speed_mps'], 0.)
        self.assertTrue(payload['decision']['emergency'])
        self.assertFalse(runtime.gap_planner.junction_committed)
        self.assertFalse(runtime.stop_line_controller.released)


if __name__ == '__main__':
    unittest.main()
