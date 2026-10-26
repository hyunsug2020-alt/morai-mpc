"""Regressions for timestamped, atomic raw scans used by tunnel matching."""
import contextlib
import io
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from lidar_corridor import LidarCorridor


def cloud(points, stamp):
    points = np.asarray(points, dtype="<f4")
    fields = [SimpleNamespace(name=name, offset=index * 4, datatype=7)
              for index, name in enumerate(("x", "y", "z"))]
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp)),
                           fields=fields, point_step=12, width=len(points), height=1,
                           data=points.tobytes())


class LidarScanSnapshotTests(unittest.TestCase):
    def test_raw_scan_retains_wall_heights_and_source_time(self):
        corridor = LidarCorridor()
        points = np.array([[4., 2., 3.5], [6., -2., .5]], np.float32)
        corridor.on_cloud(cloud(points, 123.125))
        scan = corridor.scan_snapshot()
        np.testing.assert_allclose(scan["xyz"], points + corridor.mount)
        self.assertEqual(scan["stamp"], 123.125)
        self.assertEqual(scan["sequence"], 1)
        self.assertLess(scan["age_s"], .1)
        filtered, _ = corridor.snapshot()
        self.assertEqual(len(filtered), 1)  # Elevated walls remain raw-only.
        np.testing.assert_array_equal(scan['obstacle_points'], filtered)
        self.assertFalse(scan['obstacle_points'].flags.writeable)

    def test_legacy_ingest_still_provides_xy_clearance(self):
        corridor = LidarCorridor(mount=(0., 0., 0.))
        with patch("lidar_corridor.time.time", return_value=150.):
            corridor.ingest(np.array([[8., 0., 1.]], np.float32))
        scan = corridor.scan_snapshot()
        self.assertEqual(scan["stamp"], 150.)
        path = np.column_stack((np.arange(0., 15., .5), np.zeros(30)))
        self.assertIsNotNone(corridor.blocked(path, 1.))

    def test_arrays_are_immutable_and_remain_independent_of_later_scans(self):
        corridor = LidarCorridor(mount=(0., 0., 0.))
        points = np.array([[8., 0., 1.]], np.float32)
        corridor.ingest(points, stamp=100.)
        first = corridor.scan_snapshot()
        points[:] = 999.
        with self.assertRaises(ValueError):
            first["xyz"][0, 0] = 999.
        with self.assertRaises(ValueError):
            first["ground_plane"][0] = 999.
        corridor.ingest(np.array([[9., 0., 2.]], np.float32), stamp=101.)
        np.testing.assert_array_equal(first["xyz"], [[8., 0., 1.]])
        self.assertEqual(first["sequence"], 1)
        self.assertEqual(corridor.scan_snapshot()["sequence"], 2)

    def test_stale_and_malformed_raw_scans_are_unavailable(self):
        corridor = LidarCorridor(mount=(0., 0., 0.))
        with patch("lidar_corridor.time.monotonic", return_value=20.):
            corridor.ingest(np.array([[8., 0., 1.]], np.float32), stamp=100.)
        with patch("lidar_corridor.time.monotonic", return_value=20.61):
            self.assertIsNone(corridor.scan_snapshot()["xyz"])
            self.assertIsNone(corridor.snapshot()[0])
        malformed = cloud([[8., 0., 1.]], 101.)
        malformed.fields = malformed.fields[:2]
        with contextlib.redirect_stdout(io.StringIO()):
            corridor.on_cloud(malformed)
        self.assertIsNone(corridor.scan_snapshot()["xyz"])
        self.assertEqual(corridor.scan_snapshot()["sequence"], 1)
        corridor.ingest(np.array([[8., 0., 1.]], np.float32), stamp=102.)
        self.assertIsNotNone(corridor.scan_snapshot()["xyz"])

    def test_invalid_source_stamp_cannot_refresh_a_raw_scan(self):
        corridor = LidarCorridor()
        corridor.ingest(np.array([[8., 0., 1.]], np.float32), stamp=100.)
        for stamp in (float("nan"), float("inf"), 0., -1.):
            with self.subTest(stamp=stamp):
                with self.assertRaises(ValueError):
                    corridor.ingest(np.array([[8., 0., 1.]], np.float32), stamp=stamp)
                self.assertIsNone(corridor.scan_snapshot()["xyz"])
                self.assertEqual(corridor.scan_snapshot()["sequence"], 1)

    def test_snapshot_does_not_mix_a_new_plane_with_the_previous_cloud(self):
        corridor = LidarCorridor(mount=(0., 0., 0.))
        corridor.ingest(np.array([[8., 0., 1.]], np.float32), stamp=100.)
        previous = corridor.scan_snapshot()
        entered, release = threading.Event(), threading.Event()
        errors = []

        def delayed_fit(points, fallback):
            entered.set()
            if not release.wait(2.):
                raise RuntimeError("test producer was not released")
            return np.array([0., 0., 2.])

        def produce():
            try:
                corridor.ingest(np.array([[9., 0., 2.]], np.float32), stamp=101.)
            except Exception as error:
                errors.append(error)

        with patch("lidar_corridor.fit_road_plane", side_effect=delayed_fit):
            producer = threading.Thread(target=produce)
            producer.start()
            try:
                self.assertTrue(entered.wait(1.))
                during = corridor.scan_snapshot()
                xy_during = corridor.xy_snapshot()
                self.assertEqual(xy_during["stamp"], previous["stamp"])
                self.assertEqual(xy_during["sequence"], previous["sequence"])
                np.testing.assert_array_equal(xy_during["points"], previous["obstacle_points"])
                self.assertEqual(during["stamp"], previous["stamp"])
                self.assertEqual(during["sequence"], previous["sequence"])
                np.testing.assert_array_equal(during["xyz"], previous["xyz"])
                np.testing.assert_array_equal(during["ground_plane"], previous["ground_plane"])
            finally:
                release.set()
                producer.join(2.)
        self.assertFalse(producer.is_alive())
        self.assertEqual(errors, [])
        current = corridor.scan_snapshot()
        xy = corridor.xy_snapshot()
        self.assertEqual(xy["stamp"], current["stamp"])
        self.assertEqual(xy["sequence"], current["sequence"])
        np.testing.assert_array_equal(xy["points"], current["obstacle_points"])
        self.assertEqual(current["stamp"], 101.)
        self.assertEqual(current["sequence"], 2)
        np.testing.assert_array_equal(current["xyz"], [[9., 0., 2.]])
        np.testing.assert_array_equal(current["ground_plane"], [0., 0., 2.])


if __name__ == "__main__":
    unittest.main()
