"""Final roadside nudge keeps native merge curvature and full safety checks."""
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial import cKDTree

from gui import highway_nudge, RosPathPublisher
from highway_planner import lane_change_path, local_points, world_points, geometry_speed_cap
from lane_corridor import LaneCorridor
from planning.highway.lane_map import HighwayLaneMap
from planning.highway.nudge_geometry import nudge_geometry
from test_highway_three_lane import SurveyedHighwayTests
from tunnel_runtime import swept_clearance


def stations(path):
    return np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]


class HighwayNudgeGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        SurveyedHighwayTests.setUpClass()
        cls.survey = SurveyedHighwayTests()

    def recorded_merge(self):
        ego = dict(x=73.1730293878348, y=-197.22741577917196,
                   heading_deg=-83.4095277819574, speed_kmh=42.1918382338551,
                   stamp=1791546957.7282722)
        route = local_points(self.survey.smooth[3161:3801], ego)
        native = HighwayLaneMap(self.survey.lanes, self.survey.config)
        lanes = native.candidates(ego, route)
        lane = next(l for l in lanes if l.link_id == 'A2256W000445')
        path = lane_change_path(lane.centre, 0., ego['speed_kmh']/3.6*2.5, match_heading=True)
        return ego, path, native.corridor(ego, route, lanes)

    def test_recorded_pose_merge_nudge_does_not_recreate_the_51_kmh_corner(self):
        ego, path, road = self.recorded_merge()
        full = stations(path)
        speed = ego['speed_kmh']/3.6
        at = int(np.argmin(abs(full-20.831647815514714)))
        # Recorded +0.586 m peak: a saturated demand near the common entrance,
        # a 2-second triangular ramp, 3 m corner averaging, and the origin join.
        old = np.maximum(0., .6-.6/max(12., speed*2.)*abs(full-full[at]))
        k = max(1, int(round(1.5/np.median(np.diff(full)))))
        old = np.convolve(np.r_[np.full(k, old[0]), old, np.full(k, old[-1])],
                          np.ones(2*k+1)/(2*k+1), mode='valid')
        u = np.clip(full/max(12., speed*2.), 0., 1.)
        old *= u**3*(10.-15.*u+6.*u*u)
        tangent = np.gradient(path, axis=0)
        tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        self.assertGreater(old.max(), .55)
        self.assertLess(geometry_speed_cap(path+normal*old[:, None], 25.)*3.6, 55.)
        arc = full[full < 90.]
        left, right = np.full(len(arc), 2.), np.full(len(arc), 2.)
        right[at] = .59
        with patch('gui.highway_side_gaps', return_value=(arc, left, right)) as gaps:
            proposed = highway_nudge(path, [], None, speed)
        gaps.assert_called_once()
        self.assertGreaterEqual(geometry_speed_cap(proposed, 25.)*3.6, 70.)
        published = local_points(RosPathPublisher.resample(world_points(proposed, ego)), ego)
        self.assertGreaterEqual(geometry_speed_cap(published, 25.)*3.6, 70.)
        self.assertGreaterEqual(road.clear_distance(proposed, 170.), 169.9)
        np.testing.assert_array_equal(proposed[0], path[0])
        self.assertLessEqual(np.linalg.norm(proposed-path, axis=1).max(), .600001)

    def test_empty_scan_keeps_the_original_geometry(self):
        _, path, _ = self.recorded_merge()
        np.testing.assert_array_equal(highway_nudge(path, [], None, 25.), path)

    def test_two_sided_returns_keep_local_avoidance_and_raw_clearance(self):
        path = np.column_stack((np.arange(0., 140.5, .5), np.zeros(281)))
        for side in (-1., 1.):
            points = np.array([[40., -side*1.12], [90., side*1.12]])
            proposed = highway_nudge(path, points, cKDTree(points), 12.)
            self.assertGreater(side*proposed[80, 1], .05)
            self.assertLess(side*proposed[180, 1], -.05)
            self.assertLessEqual(abs(proposed[:, 1]).max(), .6)
            self.assertGreater(swept_clearance(proposed, points, 120., lateral_margin=.15), 119.)

    def test_current_raw_collision_cannot_be_hidden_by_a_smooth_join(self):
        path = np.column_stack((np.arange(0., 100.5, .5), np.zeros(201)))
        points = np.array([[1., .8], [1.1, .8]])
        proposed = highway_nudge(path, points, cKDTree(points), 25.)
        self.assertEqual(swept_clearance(proposed, points, 100., lateral_margin=.15), 0.)

    def test_road_width_and_opposite_body_collision_still_reject_a_proposal(self):
        path = np.column_stack((np.arange(0., 140.5, .5), np.zeros(281)))
        proposed = nudge_geometry(path, [50.], [.6], 12.)
        road = LaneCorridor(path, np.full(len(path), 1.3), np.full(len(path), -1.3))
        self.assertGreater(road.clear_distance(path, 120.), 119.)
        self.assertLess(road.clear_distance(proposed, 120.), 60.)
        opposite_return = proposed[100]+[0., .9]
        self.assertLess(swept_clearance(proposed, [opposite_return], 120., lateral_margin=.15), 60.)


if __name__ == '__main__':
    unittest.main()
