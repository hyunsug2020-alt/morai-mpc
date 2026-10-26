"""Highway forecasts use the measured rear axle and the complete observed box."""
import math
import unittest

import numpy as np

from highway_planner import (geometry_speed_profile, lane_change_path,
                             target_lane_rear_conflict, timed_gap, world_points)
from highway_speed_plan import HighwaySpeedPlanner, SpeedTrajectory


FRONT_M, REAR_M, HALF_WIDTH_M = 3.845, .79, .946


def polygon(local, centre, heading_deg):
    """Independent corner construction; no production body/support helpers."""
    angle = math.radians(heading_deg)
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    return np.asarray(local, dtype=float) @ rotation.T + np.asarray(centre)


def ego_polygon(ego):
    return polygon([(FRONT_M, HALF_WIDTH_M), (-REAR_M, HALF_WIDTH_M),
                    (-REAR_M, -HALF_WIDTH_M), (FRONT_M, -HALF_WIDTH_M)],
                   (ego['x'], ego['y']), ego['heading_deg'])


def vehicle_polygon(obj):
    length, width = obj[6]/2., obj[7]/2.
    return polygon([(length, width), (-length, width), (-length, -width),
                    (length, -width)], obj[:2], obj[8])


def polygons_overlap(first, second):
    """Convex polygon oracle from edge-normal interval intersections."""
    for points in (first, second):
        edges = np.roll(points, -1, axis=0)-points
        for edge in edges:
            axis = np.array([-edge[1], edge[0]])
            a, b = first @ axis, second @ axis
            if a.max() < b.min() or b.max() < a.min():
                return False
    return True


def car(x, y=0., speed=0., uid=1, length=.4, width=.4, yaw=0., motion_heading=None):
    angle = math.radians(yaw if motion_heading is None else motion_heading)
    return (x, y, max(length, width), 'lidar_car', 'npc', speed,
            length, width, yaw, speed*math.cos(angle), speed*math.sin(angle),
            uid, 0., None, True, 0.)


def at_local(obj, ego):
    position = world_points(np.array([obj[:2]]), ego)[0]
    angle = math.radians(ego['heading_deg'])
    vx, vy = obj[9:11]
    row = list(obj)
    row[:2] = position
    row[8] += ego['heading_deg']
    row[9:11] = (vx*math.cos(angle)-vy*math.sin(angle),
                  vx*math.sin(angle)+vy*math.cos(angle))
    return tuple(row)


def stopped_trajectory(horizon=3.):
    times = np.arange(0., horizon+.01, .1)
    zero = np.zeros(len(times))
    return SpeedTrajectory(times, zero.copy(), zero.copy(), zero.copy(), 0.)


class RearAxlePhysicalFootprintTests(unittest.TestCase):
    def setUp(self):
        self.path = np.column_stack((np.arange(0., 501., .5), np.zeros(1002)))
        self.ego = dict(x=12., y=-7., heading_deg=37., speed_kmh=0., stamp=10.)

    def risk(self, obj, path=None, trajectory=None, **kwargs):
        return timed_gap(self.path if path is None else path, self.ego, [obj], 0.,
            lane_change=False, speed_trajectory=(stopped_trajectory()
                if trajectory is None else trajectory), footprint_margin_m=0., **kwargs)

    def test_front_rear_and_side_boundaries_match_rotated_polygon_oracle(self):
        # A small measured box is moved just inside/outside each real bumper.
        for side in ('front', 'rear', 'left', 'right'):
            for epsilon in (-1e-7, 1e-7):
                with self.subTest(side=side, epsilon=epsilon):
                    if side == 'front':
                        local = car(FRONT_M+.2+epsilon)
                    elif side == 'rear':
                        local = car(-REAR_M-.2-epsilon)
                    else:
                        lateral = HALF_WIDTH_M+.2+epsilon
                        local = car(1., lateral if side == 'left' else -lateral)
                    obj = at_local(local, self.ego)
                    overlap = polygons_overlap(ego_polygon(self.ego), vehicle_polygon(obj))
                    risk = self.risk(obj)
                    self.assertEqual(risk is not None, overlap)
                    if overlap:
                        self.assertEqual(risk['physical_collision_when'], 0.)

    def test_independent_polygons_cover_tilted_measured_boxes(self):
        positions = [(4., .5), (4.7, 1.2), (-.7, 1.1), (1.5, 2.), (7., 0.)]
        for yaw in (-47., 0., 23., 89.):
            for x, y in positions:
                with self.subTest(yaw=yaw, position=(x,y)):
                    obj = at_local(car(x,y,length=1.8,width=.7,yaw=yaw), self.ego)
                    overlap = polygons_overlap(ego_polygon(self.ego), vehicle_polygon(obj))
                    risk = self.risk(obj)
                    physical = risk is not None and risk.get('physical_collision_when') == 0.
                    self.assertEqual(physical, overlap)

    def test_current_rear_side_contact_is_not_hidden_by_rear_escape_rules(self):
        for position in ((-.95,0.), (-.6,1.05)):
            with self.subTest(position=position):
                obj = at_local(car(*position, speed=1., motion_heading=0.), self.ego)
                self.assertTrue(polygons_overlap(ego_polygon(self.ego), vehicle_polygon(obj)))
                risk = self.risk(obj, ignore_rear_approach=True)
                self.assertIsNotNone(risk)
                self.assertEqual(risk['physical_collision_when'], 0.)

    def test_t0_contact_uses_actual_pose_even_when_path_starts_beside_it(self):
        shifted = self.path+np.array([0.,6.])
        obj = at_local(car(4.), self.ego)
        self.assertTrue(polygons_overlap(ego_polygon(self.ego), vehicle_polygon(obj)))
        risk = self.risk(obj, path=shifted)
        self.assertIsNotNone(risk)
        self.assertEqual(risk['physical_collision_when'], 0.)

    def test_stationary_prefix_keeps_measured_heading_on_a_different_path_heading(self):
        vertical = np.column_stack((np.zeros(1002), np.arange(0.,501.,.5)))
        obj = at_local(car(4.), self.ego)
        risk = self.risk(obj, path=vertical, trajectory=stopped_trajectory().prefix(.4))
        self.assertIsNotNone(risk)
        self.assertEqual(risk['physical_collision_when'], 0.)

    def test_stopping_after_motion_keeps_the_path_heading_at_the_stop(self):
        ego = dict(x=0.,y=0.,heading_deg=0.,speed_kmh=0.,stamp=10.)
        vertical = np.column_stack((np.zeros(1002),np.arange(0.,501.,.5)))
        times = np.arange(0.,.61,.1)
        distances = np.array([0.,.3,.6,1.,1.,1.,1.])
        trajectory = SpeedTrajectory(times,distances,np.gradient(distances,.1),
                                     np.zeros(len(times)),0.)
        risk = timed_gap(vertical,ego,[car(0.,4.7)],0.,lane_change=False,
                         speed_trajectory=trajectory,footprint_margin_m=0.)
        self.assertIsNotNone(risk)
        self.assertAlmostEqual(risk['physical_collision_when'],.3)

    def test_duplicate_vertices_preserve_valid_geometry_and_invalid_paths_block(self):
        obj = at_local(car(4.),self.ego)
        expected = self.risk(obj)
        repeated = np.repeat(self.path,2,axis=0)
        self.assertEqual(self.risk(obj,path=repeated),expected)
        invalid = (np.zeros((4,2)),np.array([[0.,0.],[1.,0.],[2.,0.]]),
                   np.array([[0.,0.],[float('nan'),0.],[20.,0.]]))
        for path in invalid:
            with self.subTest(path=path.tolist()):
                self.assertIsNotNone(self.risk(obj,path=path))
                # Geometry failure remains blocked on an apparently empty road.
                self.assertIsNotNone(timed_gap(path,self.ego,[],0.,
                    speed_trajectory=stopped_trajectory(),footprint_margin_m=0.))

    def test_explicit_legacy_shift_remains_available_without_changing_default(self):
        obj = at_local(car(3.,length=.4),self.ego)
        self.assertIsNotNone(self.risk(obj))
        self.assertIsNone(self.risk(obj,body_shift_m=0.))


class RearAxleGapAndExecutionTests(unittest.TestCase):
    @staticmethod
    def change(rear_x, rear_speed, acceleration=0., goal_kmh=60.):
        ego = dict(x=0.,y=0.,heading_deg=0.,speed_kmh=60.,stamp=10.)
        line = np.column_stack((np.arange(-100.,501.),np.full(601,3.5)))
        path = lane_change_path(line[line[:,0]>=0],0.,45.,match_heading=True)
        objects = [car(rear_x,3.5,rear_speed,length=4.9,width=2.1)]
        model = HighwaySpeedPlanner(2.5,4.,12.)
        model.acceleration = acceleration
        model.motion_paths = {1:(line,np.arange(601.),100.+rear_x,rear_speed,0.,0.)}
        trajectory = model.preview(path,ego,objects,goal_kmh/3.6,14.,
            speed_profile=geometry_speed_profile(path,goal_kmh/3.6))
        kwargs = dict(object_positions=model.object_positions,object_yaws=model.object_yaws,
                      footprint_margin_m=.9)
        nominal = timed_gap(path,ego,objects,goal_kmh/3.6,speed_trajectory=trajectory,**kwargs)
        command = model.command_preview(trajectory.prefix(3.))
        executed = timed_gap(path,ego,objects,trajectory.reference_mps,
                             speed_trajectory=command,**kwargs)
        return trajectory,command,nominal,executed

    def test_rear_gap_reclaims_real_space_without_weakening_headway(self):
        _,_,safe,command_safe = self.change(-9.,60./3.6)
        self.assertIsNone(safe)
        self.assertIsNone(command_safe)
        _,_,unsafe,command_unsafe = self.change(-8.5,60./3.6)
        self.assertIsNotNone(unsafe)
        self.assertIsNotNone(command_unsafe)
        self.assertIsNone(unsafe['physical_collision_when'])

    def test_nominal_acceleration_does_not_authorize_a_slower_scalar_command(self):
        # Stored design replay: reference 70.29 km/h runs 3-4 m less in 3 s
        # than the nominal plan and loses the rear headway of a 23 m/s car.
        for acceleration in (0.,3.):
            with self.subTest(initial_acceleration=acceleration):
                trajectory,command,nominal,executed = self.change(-25.,23.,acceleration,100.)
                self.assertIsNone(nominal)
                self.assertIsNotNone(executed)
                self.assertTrue(executed['rear_approach'])
                self.assertIsNone(executed['physical_collision_when'])
                self.assertGreater(float(np.interp(3.,trajectory.times,trajectory.distances)),
                                   command.distances[-1]+2.)

    def test_fixed_rear_gate_uses_the_tilted_box_projection(self):
        ego = dict(x=12.,y=-7.,heading_deg=37.,speed_kmh=60.,stamp=10.)
        shape = vehicle_polygon(car(0.,yaw=20.,length=6.,width=2.4))
        along_half = float(np.max(np.abs(shape[:,0])))
        for epsilon in (-1e-7,1e-7):
            with self.subTest(epsilon=epsilon):
                x = -REAR_M-along_half-3.+epsilon
                obj = at_local(car(x,3.5,60./3.6,length=6.,width=2.4,
                                   yaw=20.,motion_heading=0.),ego)
                conflict = target_lane_rear_conflict(ego,[obj],3.5,60./3.6)
                self.assertEqual(conflict is not None,epsilon>0.)


class RearAxleLeaderForecastTests(unittest.TestCase):
    def test_projected_leader_support_matches_corners_and_scalar_batch_forecasts(self):
        path = np.column_stack((np.arange(0.,501.,.5),np.zeros(1002)))
        ego = dict(x=0.,y=0.,heading_deg=0.,speed_kmh=36.,stamp=10.)
        objects = [car(20.,speed=8.,length=6.,width=2.4,yaw=20.,motion_heading=0.)]
        model = HighwaySpeedPlanner()
        # A changing observed footprint heading must be projected independently
        # at every forecast time, including after a surveyed lane turn.
        model.object_yaws = lambda obj,times: np.radians(20.+5.*times)
        times = np.arange(0.,2.01,.1)
        support = model.leaders(path,ego,objects,times)[0][3]
        expected = []
        for yaw in 20.+5.*times:
            corners = vehicle_polygon(car(0.,yaw=yaw,length=6.,width=2.4))
            expected.append(float(np.max(np.abs(corners[:,0]))))
        np.testing.assert_allclose(support,expected,rtol=0.,atol=1e-12)
        goals = [10.,15.,20.]
        batched = model.preview_many(path,ego,objects,goals,2.)
        for goal,batch in zip(goals,batched):
            scalar = model.preview(path,ego,objects,goal,2.)
            for field in ('distances','speeds','accelerations'):
                np.testing.assert_allclose(getattr(batch,field),getattr(scalar,field),
                                           rtol=0.,atol=1e-12)
        legacy = HighwaySpeedPlanner(body_shift_m=0.)
        legacy.object_yaws = model.object_yaws
        old = legacy.preview(path,ego,objects,20.,2.)
        self.assertTrue(np.any(batched[-1].speeds<old.speeds-1e-7))


if __name__ == '__main__':
    unittest.main()
