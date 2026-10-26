import math
import unittest

import numpy as np

import lead_follow as lf


def straight(length=80., step=.5):
    x = np.arange(0., length, step)
    return np.column_stack((x, np.zeros_like(x)))


def rear_face(dist, lat=0., width=1.9, spacing=.1):
    y = np.arange(lat - width / 2, lat + width / 2 + 1e-6, spacing)
    return np.column_stack((np.full_like(y, dist), y))


class FollowCapTests(unittest.TestCase):
    def test_standing_lead_stops_at_standstill_gap(self):
        self.assertEqual(lf.follow_cap_mps(lf.STANDSTILL_GAP_M, 0.), 0.)
        self.assertEqual(lf.follow_cap_mps(1., 0.), 0.)

    def test_cap_grows_with_gap(self):
        caps = [lf.follow_cap_mps(g, 0.) for g in (6., 10., 20., 40., 60.)]
        self.assertEqual(caps, sorted(caps))
        # 55 km/h can still stop behind a car seen ~59 m ahead
        self.assertGreater(lf.follow_cap_mps(59. - lf.FRONT_M, 0.), 55. / 3.6 - .5)

    def test_cap_respects_stopping_distance(self):
        for gap in (8., 15., 30., 50.):
            v = lf.follow_cap_mps(gap, 0.)
            need = v * lf.REACTION_S + v * v / (2 * lf.BRAKE_MPS2) + lf.STANDSTILL_GAP_M
            self.assertLessEqual(need, gap + 1e-6)

    def test_moving_lead_at_time_gap_is_not_capped(self):
        # 50 km/h lead, 30 m behind it: following distance about 1.9 s is fine
        self.assertGreater(lf.follow_cap_mps(30., 50. / 3.6), 50. / 3.6)


class LidarLeadTests(unittest.TestCase):
    def test_dense_scan_projection_preserves_station_lateral_and_endpoints(self):
        x = np.linspace(-1., 81., 4097)
        points = np.column_stack((x, np.sin(x)))
        path = straight(80.5)
        station, lateral, index = lf.path_coordinates(path, points, horizon_m=80.)
        inside = (points[index, 0] >= 0.) & (points[index, 0] <= 80.)
        np.testing.assert_allclose(station[inside], points[index[inside], 0], atol=1e-12)
        np.testing.assert_allclose(lateral, points[index, 1], atol=1e-12)
        self.assertTrue(np.isnan(station[~inside]).all())

    def test_dense_curved_scan_projection_preserves_both_sides_of_turn(self):
        angle = np.linspace(0., 1.2, 241)
        path = np.column_stack((25.*np.sin(angle), 25.*(1.-np.cos(angle))))
        segment = np.diff(path, axis=0)
        length = np.linalg.norm(segment, axis=1)
        normal = np.column_stack((-segment[:, 1], segment[:, 0]))/length[:, None]
        midpoint = .5*(path[:-1]+path[1:])
        points = np.vstack([midpoint+offset*normal for offset in (-.8, -.4, 0., .4, .8)])
        station, lateral, index = lf.path_coordinates(path, points)
        expected_station = np.tile(np.cumsum(length)-.5*length, 5)
        expected_lateral = np.repeat([-.8, -.4, 0., .4, .8], len(length))
        np.testing.assert_allclose(station, expected_station[index], atol=1e-9)
        np.testing.assert_allclose(lateral, expected_lateral[index], atol=1e-9)

    def test_car_ahead_found(self):
        lead = lf.lidar_lead(straight(), rear_face(30.))
        self.assertIsNotNone(lead)
        self.assertAlmostEqual(lead["dist_m"], 30., delta=.3)

    def test_far_car_found(self):
        self.assertIsNotNone(lf.lidar_lead(straight(), rear_face(55.)))

    def test_guardrail_beside_the_body_ignored(self):
        x = np.arange(3., 60., .3)
        rail = np.column_stack((x, np.full_like(x, 1.2)))
        self.assertIsNone(lf.lidar_lead(straight(), rail))
        rail_close = np.column_stack((x, np.full_like(x, .9)))     # 3 cm body gap, still a line
        self.assertIsNone(lf.lidar_lead(straight(), rail_close))

    def test_adjacent_lane_car_ignored(self):
        self.assertIsNone(lf.lidar_lead(straight(), rear_face(20., lat=3.5)))

    def test_partly_overlapping_car_found(self):
        # centre 1.5 m to the side: 0.4 m of it in front of our body
        self.assertIsNotNone(lf.lidar_lead(straight(), rear_face(20., lat=1.5)))

    def test_body_aligned_path_recovers_heading_bias(self):
        # map path drawn 3 deg off (localization heading bias); the car is straight ahead
        angle = math.radians(3.)
        path = straight() @ np.array([[math.cos(angle), math.sin(angle)], [-math.sin(angle), math.cos(angle)]])
        pts = rear_face(40.)
        self.assertIsNone(lf.lidar_lead(path, pts))
        aligned = lf.body_aligned_path(path, lf.path_heading_rad(path))
        lead = lf.lidar_lead(aligned, pts)
        self.assertIsNotNone(lead)
        self.assertAlmostEqual(lead["dist_m"], 40., delta=.5)


class TrackerTests(unittest.TestCase):
    def test_speed_of_slower_lead(self):
        tracker, ego_v, lead_v = lf.LeadTracker(), 15., 10.
        gap, state = 40., None
        for k in range(20):
            t = k * .1
            state = tracker.update(t, ego_v, dict(dist_m=gap), (), scan_stamp=t)
            gap += (lead_v - ego_v) * .1
        self.assertTrue(state["known"])
        self.assertAlmostEqual(state["v_lead"], lead_v, delta=.5)
        self.assertTrue(state["confirmed"])

    def test_unknown_speed_treated_as_standing(self):
        tracker = lf.LeadTracker()
        tracker.update(0., 10., dict(dist_m=20.), (), scan_stamp=0.)
        state = tracker.update(.1, 10., dict(dist_m=19.), (), scan_stamp=.1)
        self.assertFalse(state["known"])
        self.assertAlmostEqual(lf.lead_cap(state, 10.), lf.follow_cap_mps(state["gap_m"], 0.))

    def test_single_far_scan_waits_for_confirmation(self):
        tracker = lf.LeadTracker()
        state = tracker.update(0., 14., dict(dist_m=50.), (), scan_stamp=0.)
        self.assertFalse(state["confirmed"])
        self.assertIsNone(lf.lead_cap(state, 14.))
        near = lf.LeadTracker().update(0., 14., dict(dist_m=12.), (), scan_stamp=0.)
        self.assertIsNotNone(lf.lead_cap(near, 14.))      # too close to wait

    def test_flicker_hold(self):
        tracker = lf.LeadTracker()
        for k in range(5):
            tracker.update(k * .1, 0., dict(dist_m=8.), (), scan_stamp=k * .1)
        held = tracker.update(.5, 0., None, ())
        self.assertTrue(held["held"])
        self.assertAlmostEqual(held["dist_m"], 8., delta=.1)
        self.assertIsNone(tracker.update(1.2, 0., None, ()))

    def test_object_velocity_used(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        car = (25., .2, 4.6, "fusion_npc", "npc", 8., 4.6, 1.9, 0., 8., 0., 7, 0., None)
        leads = lf.object_leads([car], ego, straight())
        self.assertEqual(len(leads), 1)
        self.assertAlmostEqual(leads[0]["dist_m"], 25. - 2.3, delta=.3)
        state = lf.LeadTracker().update(0., 12., None, leads)
        self.assertTrue(state["known"])
        self.assertAlmostEqual(state["v_lead"], 8., delta=.1)

    def test_oncoming_and_side_cars_ignored(self):
        ego = dict(x=0., y=0., heading_deg=0.)
        oncoming = (25., .3, 4.6, "fusion_npc", "npc", 10., 4.6, 1.9, 180., -10., 0., 8, 0., None)
        side = (25., 3.5, 4.6, "fusion_npc", "npc", 5., 4.6, 1.9, 0., 5., 0., 9, 0., None)
        self.assertEqual(lf.object_leads([oncoming, side], ego, straight()), [])


if __name__ == "__main__":
    unittest.main()
