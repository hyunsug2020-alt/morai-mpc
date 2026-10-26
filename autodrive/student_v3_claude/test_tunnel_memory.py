"""Measured free rays clear ghosts; absence alone never clears blind barriers."""
import unittest

import numpy as np

from highway_planner import NearFieldMemory
from tunnel_memory import TunnelNearFieldMemory


class TunnelMemoryTests(unittest.TestCase):
    mount = np.array([1.05, 0., 1.35])
    points = np.array([(x, y) for x in (4.8, 5.2) for y in (-.4, -.2, .2, .4)])

    def scan(self, stamp, *, barrier=False, ray_height=.76, both=True, age=.01):
        angles = np.linspace(-3.14, 3.14, 120)
        xyz = [np.column_stack((30*np.cos(angles), 30*np.sin(angles), np.zeros(len(angles))))]
        for point in self.points:
            for delta in ((-.012, .012) if both else (.012,)):
                target = np.array([point[0], point[1]+delta, ray_height])
                xyz.append((self.mount+2.*(target-self.mount))[None, :])
        if barrier:
            xyz.append(np.column_stack((self.points, np.full(len(self.points), .8))))
        return dict(stamp=stamp, age_s=age, xyz=np.vstack(xyz), ground_plane=np.zeros(3))

    def step(self, memory, stamp, **kwargs):
        pose = dict(stamp=stamp, x=0., y=0., heading_deg=0.)
        memory.observe_scan(self.scan(stamp, **kwargs), pose, self.mount)
        return memory.update(self.points if kwargs.get('barrier') else [], pose, stamp)

    def seeded(self):
        memory = TunnelNearFieldMemory()
        self.step(memory, 100., barrier=True)
        self.step(memory, 100.1, barrier=True)
        self.assertTrue(memory.cells)
        self.assertTrue(memory.heights)
        self.assertTrue(all(row[1] >= 2 for row in memory.cells.values()))
        return memory

    def test_two_fresh_scans_through_the_recorded_face_clear_it(self):
        memory = self.seeded()
        self.step(memory, 100.2)
        self.assertTrue(memory.cells)
        self.step(memory, 100.3)
        self.assertFalse(memory.cells)
        self.assertGreater(memory.cleared_n, 0)

    def test_visible_barrier_vetoes_free_neighbors(self):
        memory = self.seeded()
        for stamp in (100.2, 100.3, 100.4):
            self.step(memory, stamp, barrier=True)
        self.assertTrue(memory.cells)
        self.assertEqual(memory.cleared_n, 0)

    def test_free_cell_cannot_be_recreated_by_the_other_memory_snapshot(self):
        memory = self.seeded()
        self.step(memory, 100.2)
        self.step(memory, 100.3)
        for stamp in (100.4, 100.5):
            pose = dict(stamp=stamp, x=0., y=0., heading_deg=0.)
            memory.observe_scan(self.scan(stamp), pose, self.mount)
            memory.update(self.points, pose, stamp)
            self.assertFalse(memory.cells)
        self.step(memory, 100.6, barrier=True)
        self.assertTrue(memory.cells)

    def test_low_barrier_hidden_below_the_beams_has_no_timeout(self):
        memory = self.seeded()
        for stamp in (100.2, 101., 110., 160.):
            self.step(memory, stamp, ray_height=1.1)
        self.assertTrue(memory.cells)
        self.assertEqual(memory.cleared_n, 0)

    def test_one_side_or_one_source_frame_cannot_clear_a_barrier(self):
        memory = self.seeded()
        for stamp in (100.2, 100.3, 100.4):
            self.step(memory, stamp, both=False)
        self.assertTrue(memory.cells)
        self.step(memory, 100.5)
        self.step(memory, 100.5)
        self.assertTrue(memory.cells)

    def test_replayed_constraints_do_not_renew_measurement_support(self):
        memory = self.seeded()
        before = {key: row[0] for key, row in memory.cells.items()}
        pose = dict(stamp=100.3, x=0., y=0., heading_deg=0.)
        memory.observe_scan(self.scan(100.3, ray_height=1.1), pose, self.mount)
        output = memory.update(self.points, pose, 100.3)
        self.assertTrue(len(output))
        self.assertEqual(before, {key: row[0] for key, row in memory.cells.items()})

    def test_stale_scan_and_clock_reset_cannot_prove_free_space(self):
        for stamp, age in ((100.2, .26), (100.05, .01)):
            memory = self.seeded()
            self.step(memory, stamp, age=age)
            if stamp > 100.1:
                self.assertTrue(memory.cells)
            self.assertEqual(memory.cleared_n, 0)
            self.assertIsNone(memory.geometry)

    def test_without_physical_tunnel_observation_behavior_is_unchanged(self):
        old = NearFieldMemory(retain_measured_points=True, hold_blind=True)
        new = TunnelNearFieldMemory()
        for stamp, points in ((100., self.points), (100.1, self.points), (102., []), (110., [])):
            pose = dict(x=0., y=0., heading_deg=0.)
            np.testing.assert_array_equal(old.update(points, pose, stamp), new.update(points, pose, stamp))
            self.assertEqual(set(old.cells), set(new.cells))


if __name__ == '__main__':
    unittest.main()
