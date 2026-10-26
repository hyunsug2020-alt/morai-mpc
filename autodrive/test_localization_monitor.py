"""Regression checks for concurrent ROS status updates and status-file behavior."""
from collections import deque
import csv
import importlib.util
import itertools
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location('localization_monitor_tested', Path(__file__).with_name('localization_monitor.py'))
monitor = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'rospy': SimpleNamespace(),
                             'morai_msgs.msg': SimpleNamespace(EgoVehicleStatus=object)}):
    spec.loader.exec_module(monitor)


class YieldingDeque(deque):
    def __iter__(self):
        for index, value in enumerate(super().__iter__()):
            if index == 0:
                time.sleep(0)
            yield value


class LocalizationMonitorTests(unittest.TestCase):
    def test_empty_and_bounded_history_keep_a_coherent_latest_sample(self):
        history = monitor.EgoStatusHistory()
        self.assertEqual(history.snapshot(), ((), None, None))
        with patch.object(monitor.time, 'monotonic', side_effect=range(250)):
            for sequence in range(250):
                history.record(SimpleNamespace(sequence=sequence))
        stamps, message, received = history.snapshot()
        self.assertEqual(stamps, tuple(range(50, 250)))
        self.assertEqual(message.sequence, received)
        self.assertEqual(received, 249)

    def test_callback_updates_cannot_mutate_an_in_progress_history_snapshot(self):
        with patch.object(monitor, 'deque', YieldingDeque):
            history = monitor.EgoStatusHistory()
        failures = []
        counter = itertools.count(1)

        def producer():
            try:
                for sequence in range(1, 30001):
                    history.record(SimpleNamespace(sequence=sequence))
                    if sequence % 32 == 0:
                        time.sleep(0)
            except BaseException as exc:
                failures.append(exc)

        with patch.object(monitor.time, 'monotonic', side_effect=lambda: next(counter)):
            thread = threading.Thread(target=producer)
            thread.start()
            reads = 0
            try:
                while thread.is_alive() or reads < 500:
                    stamps, message, received = history.snapshot()
                    if message is not None:
                        self.assertLessEqual(len(stamps), 200)
                        self.assertEqual(stamps[-1], received)
                        self.assertEqual(message.sequence, received)
                        self.assertEqual(stamps, tuple(range(received-len(stamps)+1, received+1)))
                    reads += 1
            finally:
                thread.join(timeout=5)
        self.assertFalse(thread.is_alive())
        self.assertFalse(failures, failures)
        self.assertEqual(history.snapshot()[1].sequence, 30000)

    def run_monitor_once(self, *, age, publish=True):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            route, status, trace = [root/name for name in ('route.json', 'status.json', 'trace.csv')]
            route.write_text(json.dumps(dict(waypoints=[dict(x=3., y=4.), dict(x=30., y=40.)])))
            message = SimpleNamespace(position=SimpleNamespace(x=3., y=4.),
                velocity=SimpleNamespace(x=10.), heading=20., wheel_angle=1.2)
            fake_ros = SimpleNamespace(init_node=Mock(), is_shutdown=Mock(side_effect=[False, True]),
                Subscriber=Mock(side_effect=lambda topic, cls, callback, **kwargs: callback(message) if publish else None))
            clock = [100., 100.+age] if publish else [100.+age]
            with patch.object(monitor, 'rospy', fake_ros), \
                    patch.object(monitor.time, 'monotonic', side_effect=clock), \
                    patch.object(monitor.time, 'sleep'), \
                    patch.object(sys, 'argv', ['monitor', '--route', str(route), '--status', str(status), '--trace', str(trace)]):
                monitor.main()
            value = json.loads(status.read_text())
            with trace.open() as source:
                rows = list(csv.reader(source))
            self.assertFalse(Path(str(status)+'.tmp').exists())
            self.assertEqual(fake_ros.Subscriber.call_args.args[0], '/localization/ego_status')
            return value, rows

    def test_fresh_sample_preserves_status_and_trace_contract(self):
        status, rows = self.run_monitor_once(age=.1)
        self.assertEqual(status['ego_hz'], .5)
        self.assertEqual(status['ego']['speed_kmh'], 36.)
        self.assertEqual(status['ego']['age_s'], .1)
        self.assertEqual(status['path'], dict(nearest_idx=0, distance_m=0.))
        self.assertEqual(len(rows), 2)

    def test_missing_or_stale_sample_does_not_publish_a_current_pose(self):
        for publish, age in ((False, .1), (True, 1.001), (True, 2.001)):
            with self.subTest(publish=publish, age=age):
                status, rows = self.run_monitor_once(publish=publish, age=age)
                self.assertNotIn('ego', status)
                self.assertNotIn('path', status)
                self.assertEqual(len(rows), 1)


if __name__ == '__main__':
    unittest.main()
