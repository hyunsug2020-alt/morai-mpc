"""Highway mission, measured rear gaps and near-field blindness regressions."""
import json
import math
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from highway_planner import HighwayPlanner, NearFieldMemory, geometry_speed_cap, lane_change_path, local_points, timed_gap, world_points
from lidar_vehicle_detection import VehicleTracks, aligned_scan, tracked_scan_expired
from ego_body import EGO_CENTRE_M, EGO_FRONT_M
from highway_speed_plan import SpeedTrajectory
from test_highway_rear_axle import ego_polygon, polygons_overlap, vehicle_polygon


class HighwayTests(unittest.TestCase):
    def setUp(self):
        self.route = np.column_stack((np.arange(0., 801., .5), np.zeros(1602)))
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=90.)
        self.planner = HighwayPlanner(dict(from_idx=0, to_idx=1500, kmh=90.), self.route)

    @staticmethod
    def car(x, y, speed, valid=True):
        return (x, y, 4.9, "lidar_car", "npc", speed, 4.9, 2.1, 0., speed, 0., 500000, 0., None, valid)

    def decide(self, objects=(), rear=None):
        return self.planner.decide(0, self.route[:640], self.ego, objects, {1:3.5},
            lambda p,h:h, lambda p,h:h, rear_unclassified=rear)

    def test_fast_rear_vehicle_defers_change_without_stopping(self):
        result = self.decide([self.car(-35., 3.5, 35.)])
        self.assertEqual(result['status']['phase'], 'WAIT_GAP')
        self.assertGreater(result['target_mps'], 20.)
        self.assertFalse(result['emergency'])

    def test_rear_gap_uses_measured_motion_when_obb_has_pi_ambiguity(self):
        # This synthetic gap was specified relative to the old body centre.
        rear = list(self.car(-10.+EGO_CENTRE_M, 0., 26.))
        rear[8] = 180.  # Same rectangle; its detected front/back axis is ambiguous.
        risk = timed_gap(self.route[:640], self.ego, [tuple(rear)], 25., horizon_s=.5)
        self.assertIsNotNone(risk)
        self.assertEqual(risk['when'], 0.)
        self.assertIsNone(risk['collision_when'])

    def test_lead_headway_uses_measured_motion_with_pi_ambiguous_obb(self):
        lead = list(self.car(22., 0., 10.))
        lead[8] = 180.
        ego = dict(self.ego, speed_kmh=54.)
        result = self.planner.decide(0, self.route[:640], ego, [tuple(lead)], {},
                                     lambda path, horizon: horizon, lambda path, horizon: horizon)
        # The measured 10 m/s lead bounds the 25 m/s cruise by the shared headway rule.
        from highway_speed_plan import GAP_GAIN, HEADWAY_S, MIN_GAP_M
        gap = 22. - EGO_FRONT_M - 4.9/2.
        headway_cap = 10. + GAP_GAIN*(gap - max(MIN_GAP_M, 15.*HEADWAY_S))
        self.assertLess(result['target_mps'], headway_cap + .01)
        self.assertLess(result['target_mps'], 14.)
        self.assertFalse(result['emergency'])

    def test_motion_direction_does_not_rotate_collision_box(self):
        # Keep the synthetic box level with the physical body centre.
        car = list(self.car(EGO_CENTRE_M, 3.6, 25.))
        car[8] = 90.  # This measured footprint extends into the ego's lane.
        risk = timed_gap(self.route[:640], self.ego, [tuple(car)], 25., horizon_s=.5)
        self.assertIsNotNone(risk)
        self.assertEqual(risk['collision_when'], 0.)

    def test_stationary_unknown_motion_still_preserves_imminent_collision(self):
        car = list(self.car(4., 0., 0.))
        car[8] = 180.
        result = self.decide([tuple(car)])
        self.assertTrue(result['emergency'])
        self.assertEqual(result['target_mps'], 0.)

    def test_rear_vehicle_in_current_lane_does_not_stop(self):
        result = self.decide([self.car(-40., 0., 30.)])
        self.assertFalse(result['emergency'])
        self.assertGreater(result['target_mps'], 20.)

    def test_two_lane_changes_complete_with_checked_return(self):
        first=self.decide()
        self.assertEqual(first['status']['phase'],'CHANGE_OUT')
        ego=dict(self.ego,x=125.,y=3.5)
        route=self.route[:640]-[125.,3.5]
        a=self.planner.decide(250,route,ego,[],{1:3.5},lambda p,h:h,lambda p,h:h)
        self.assertEqual(a['status']['lane_changes'],1)
        ego['x']=155.;route=self.route[310:950]-[155.,3.5]
        b=self.planner.decide(310,route,ego,[],{1:3.5},lambda p,h:h,lambda p,h:h)
        self.assertEqual(b['status']['phase'],'CHANGE_BACK')
        ego.update(x=280.,y=0.);route=self.route[560:1200]-[280.,0.]
        c=self.planner.decide(560,route,ego,[],{1:3.5},lambda p,h:h,lambda p,h:h)
        self.assertEqual(c['status']['lane_changes'],2)
        self.assertEqual(c['status']['phase'],'DONE')

    def test_minimum_goal_does_not_disable_a_further_safe_overtake(self):
        self.planner.phase='DONE';self.planner.completed=2
        result=self.decide([self.car(65.,0.,10.)])
        self.assertTrue(result['status']['further_overtake_requested'])
        self.assertEqual(result['status']['phase'],'CHANGE_OUT')
        self.assertGreaterEqual(result['target_mps']*3.6,50.)
        self.assertFalse(result['emergency'])

    def test_clear_done_lane_does_not_change_without_a_reason(self):
        self.planner.phase='DONE';self.planner.completed=2
        result=self.decide()
        self.assertEqual(result['status']['phase'],'DONE')
        self.assertFalse(result['status']['further_overtake_requested'])
        self.assertGreaterEqual(result['target_mps']*3.6,50.)

    def test_parallel_side_car_without_merge_does_not_reduce_cruise_speed(self):
        self.planner.phase='DONE';self.planner.completed=2
        result=self.planner.decide(0,self.route[:640],self.ego,[self.car(15.,3.5,8.)],{},
                                   lambda p,h:h,lambda p,h:h)
        self.assertAlmostEqual(result['target_mps'],25.)
        self.assertIsNone(result['status']['side_traffic'])
        self.assertFalse(result['emergency'])

    def test_unknown_front_car_in_current_lane_does_not_veto_free_overtaking_lane(self):
        from gui import static_clear_distance
        self.planner.phase='DONE';self.planner.completed=2
        objects=[self.car(65.,0.,0.,valid=False)]
        result=self.planner.decide(0,self.route[:640],self.ego,objects,{1:3.5},
                                   lambda p,h:h,lambda p,h:static_clear_distance(p,self.ego,objects,h))
        self.assertEqual(result['status']['phase'],'CHANGE_OUT')
        self.assertTrue(any(c['accepted'] for c in result['status']['candidate_checks']))
        self.assertFalse(result['emergency'])

    def test_optional_return_does_not_rejoin_behind_slow_front_vehicle(self):
        self.planner.phase='HOLD';self.planner.completed=1
        self.planner.target_offset=3.5;self.planner.lane_side=1;self.planner.hold_station=1.
        ego=dict(self.ego,x=100.,y=3.5)
        route=self.route[200:840]-[100.,3.5]
        result=self.planner.decide(200,route,ego,[self.car(165.,0.,8.)],{1:3.5},
                                   lambda p,h:h,lambda p,h:h)
        self.assertEqual(result['status']['phase'],'HOLD')
        self.assertAlmostEqual(result['target_mps'],25.)
        self.assertTrue(any('slower return-lane leader' in c['reason'] for c in result['status']['candidate_checks']))

    def test_additional_overtake_still_requires_return_after_completion(self):
        self.planner.phase='DONE';self.planner.completed=2
        self.assertEqual(self.decide([self.car(65.,0.,10.)])['status']['phase'],'CHANGE_OUT')
        ego=dict(self.ego,x=125.,y=3.5)
        result=self.planner.decide(250,self.route[250:890]-[125.,3.5],ego,[],{1:3.5},
                                  lambda p,h:h,lambda p,h:h)
        self.assertEqual(result['status']['lane_changes'],3)
        self.assertEqual(result['status']['phase'],'HOLD')
        ego['x']=155.
        result=self.planner.decide(310,self.route[310:950]-[155.,3.5],ego,[],{1:3.5},
                                  lambda p,h:h,lambda p,h:h)
        self.assertEqual(result['status']['phase'],'CHANGE_BACK')

    def test_late_overtake_keeps_current_lane_without_room_for_return(self):
        self.planner.phase='DONE';self.planner.completed=2
        self.planner.change_until=360
        result=self.decide([self.car(65.,0.,10.)])
        self.assertEqual(result['status']['phase'],'DONE')
        self.assertTrue(all(not c['accepted'] for c in result['status']['candidate_checks']))

    def test_further_change_from_neighbor_returns_one_lane_without_offset_accumulation(self):
        self.planner.phase='DONE';self.planner.completed=3;self.planner.target_offset=3.5
        ego=dict(self.ego,x=125.,y=3.5)
        route=self.route[250:890]-[125.,3.5]
        result=self.planner.decide(250,route,ego,[self.car(190.,3.5,10.)],
            {1:3.5,-1:3.5},lambda p,h:h,lambda p,h:h)
        self.assertEqual(result['status']['phase'],'CHANGE_OUT')
        self.assertEqual(result['status']['offset_m'],0.)
        self.assertEqual([c['offset_m'] for c in result['status']['candidate_checks']],[0.])
        self.assertFalse(result['emergency'])

    def test_mission_end_is_reached_below_the_60_kmh_limit(self):
        from highway_planner import EXIT_SPEED_MPS
        planner=HighwayPlanner(dict(from_idx=0,to_idx=400,kmh=90.),self.route)
        planner.phase='DONE';planner.completed=2
        far=planner.decide(0,self.route[:640],dict(self.ego,speed_kmh=90.),[],{},lambda p,h:h,lambda p,h:h)
        self.assertGreater(far['target_mps'],80./3.6)
        near=planner.decide(390,self.route[390:1030]-self.route[390],dict(self.ego,speed_kmh=60.),[],{},
                            lambda p,h:h,lambda p,h:h)
        self.assertLessEqual(near['target_mps'],EXIT_SPEED_MPS+1e-6)

    def test_map_corridor_alone_never_stops_on_the_highway(self):
        # Live 2026-10-03: 0.7 m off centre made the corridor "3 m" and the car
        # parked on the highway. The lane rule slows the car; it never stops it.
        from highway_planner import HIGHWAY_MAP_FLOOR_MPS
        self.planner.phase='DONE';self.planner.completed=2
        result=self.planner.decide(0,self.route[:640],dict(self.ego,speed_kmh=0.),[],{},
            lambda p,h:3.183,lambda p,h:h)
        self.assertAlmostEqual(result['target_mps'],HIGHWAY_MAP_FLOOR_MPS)
        self.assertFalse(result['emergency'])
        self.assertEqual(result['status']['clearance_components_m']['map'],3.183)
        # Floor raised to 50 km/h (user: never < 50 on the highway; auto runs 109-114),
        # so the lane rule alone no longer drops us below the minimum.
        self.assertGreaterEqual(result['target_mps'],50./3.6-1e-6)

    def test_cruising_rear_approach_is_not_a_braking_order_but_still_vetoes_lane_entry(self):
        ego=dict(self.ego,speed_kmh=0.)
        # Parallel body clearance is positive; only admission headway/shell
        # clearance fails. A real future body contact has a separate veto below.
        rear=self.car(-5.,2.2,20.)
        admission=timed_gap(self.route[:640],ego,[rear],25.)
        self.assertIsNotNone(admission)
        self.assertIsNone(admission['physical_collision_when'])
        self.assertIsNone(timed_gap(self.route[:640],ego,[rear],25.,lane_change=False))
        self.planner.phase='DONE';self.planner.completed=2
        result=self.planner.decide(0,self.route[:640],ego,[rear],{1:3.5},lambda p,h:h,lambda p,h:h)
        self.assertFalse(result['emergency'])
        self.assertGreater(result['target_mps']*3.6,50.)

    def test_cruising_body_overlap_and_frontal_collision_are_still_checked(self):
        ego=dict(self.ego,speed_kmh=0.)
        # The old -4 m synthetic rear overlap was body-centre-relative.
        for car in (self.car(-4.+EGO_CENTRE_M,0.,20.),self.car(4.,0.,0.)):
            with self.subTest(position=car[:2]):
                risk=timed_gap(self.route[:640],ego,[car],25.,lane_change=False)
                self.assertIsNotNone(risk)
                self.assertEqual(risk['collision_when'],0.)

    def test_fast_rear_future_body_contact_is_not_a_headway_only_exception(self):
        ego=dict(self.ego,speed_kmh=0.)
        rear=self.car(-5.,1.9,20.)
        risk=timed_gap(self.route[:640],ego,[rear],25.,lane_change=False)
        self.assertIsNotNone(risk)
        self.assertGreater(risk['physical_collision_when'],0.)
        # At the first 0.1 s sample the rear car catches the real bumper; its
        # lateral offset 1.9 m is inside the two bodies' combined half-width.
        other=list(rear);other[0]+=20.*.1
        moved=dict(ego,x=.006)
        self.assertTrue(polygons_overlap(ego_polygon(moved),vehicle_polygon(other)))
        self.assertAlmostEqual(risk['physical_collision_when'],.1)

    def test_unclassified_rear_returns_veto_return(self):
        self.planner.phase='HOLD';self.planner.completed=1;self.planner.target_offset=3.5;self.planner.hold_station=1.
        ego=dict(self.ego,x=30.,y=3.5)
        r=self.planner.decide(60,self.route[60:700]-[30.,3.5],ego,[],{1:3.5},
            lambda p,h:h,lambda p,h:h,rear_unclassified=np.array([[-20.,-3.5]]))
        self.assertEqual(r['status']['phase'],'HOLD')
        self.assertGreater(r['target_mps'],20.)

    def test_slow_adjacent_car_easing_into_our_lane_is_not_ignored(self):
        # Auto run 51 motivates the drift check. This synthetic 35 m placement
        # has an actual crossing-body witness, rather than the old long rear
        # shell touching a car already cleared by the real bumper.
        ego = dict(self.ego, speed_kmh=74.)
        def car(vy):
            return (35., 3.5, 4.9, "lidar_car", "npc", 11., 4.9, 2.1, math.degrees(math.atan2(vy, 11.)),
                    11., vy, 500001, 0., None, True)
        times=np.arange(0.,7.01,.1)
        trajectory=SpeedTrajectory(times,times*74./3.6,np.full(len(times),74./3.6),
                                   np.zeros(len(times)),20.5)
        self.assertIsNone(timed_gap(self.route[:640],ego,[car(0.)],20.5,lane_change=False,
                                   speed_trajectory=trajectory))
        drifting=car(-.4)
        risk=timed_gap(self.route[:640],ego,[drifting],20.5,lane_change=False,
                       speed_trajectory=trajectory)
        self.assertIsNotNone(risk)
        self.assertIsNotNone(risk['physical_collision_when'])
        when=risk['physical_collision_when']
        other=list(drifting);other[:2]=np.asarray(drifting[:2])+when*np.asarray(drifting[9:11])
        moved=dict(ego,x=when*74./3.6)
        self.assertTrue(polygons_overlap(ego_polygon(moved),vehicle_polygon(other)))

    def test_front_traffic_and_imminent_crossing_keep_safety(self):
        lead=self.decide([self.car(12.,0.,10.)])
        self.assertLess(lead['target_mps'],25.)
        self.assertTrue(lead['emergency'])

    def test_distant_curvature_uses_braking_envelope(self):
        path=self.route[:640].copy();path[400:,1]=3.*np.sin((path[400:,0]-200.)/10.)
        self.assertGreater(geometry_speed_cap(path,25.),24.)

    def test_blind_nearfield_is_retained_until_passed(self):
        memory=NearFieldMemory();points=np.array([[5.,-.2],[5.,0.],[5.,.2],[5.4,0.]])
        memory.update(points,self.ego,1.);memory.update(points,self.ego,1.1)
        retained=memory.update(np.empty((0,2)),dict(self.ego,x=1.),1.3)
        self.assertGreater(len(retained),0)
        passed=memory.update(np.empty((0,2)),dict(self.ego,x=9.),1.4)
        self.assertEqual(len(passed),0)

    def test_parked_car_does_not_keep_blind_returns_forever(self):
        # 2026-10-03 highway: retained cells never expired while stopped and
        # held a free lane "blocked" at 3 m for minutes.
        memory = NearFieldMemory(retain_measured_points=True)
        points = np.array([[5., -.2], [5., 0.], [5., .2], [5.4, 0.]])
        memory.update(points, self.ego, 1.)
        memory.update(points, self.ego, 1.1)
        self.assertGreater(len(memory.update(np.empty((0, 2)), self.ego, 1.3)), 0)
        stale = memory.update(np.empty((0, 2)), self.ego, 1.1 + NearFieldMemory.MAX_BLIND_AGE_S + .1)
        self.assertEqual(len(stale), 0)

    def test_measured_retention_keeps_roadside_cone_outside_body(self):
        from tunnel_runtime import swept_clearance
        from gui import HIGHWAY_SIDE_MARGIN_M
        memory = NearFieldMemory(retain_measured_points=True)
        cone = np.array([[5., -1.24], [5.1, -1.26], [5.2, -1.24], [5.3, -1.25]])
        memory.update(cone, self.ego, 1.)
        memory.update(cone, self.ego, 1.1)
        retained = memory.update(np.empty((0, 2)), self.ego, 1.3)
        self.assertGreater(len(retained), 0)
        self.assertTrue(np.all(np.abs(retained[:, 1]) > 1.2))
        path = np.column_stack((np.arange(0., 60., .5), np.zeros(120)))
        clear = swept_clearance(path, retained, 50., extra_margin=.25,
                                lateral_margin=HIGHWAY_SIDE_MARGIN_M)
        self.assertEqual(clear, 50.)

    def test_moving_vehicle_clears_returns_saved_before_velocity_warmup(self):
        memory = NearFieldMemory()
        points = np.array([[5., -.2], [5., 0.], [5., .2], [5.4, 0.]])
        memory.update(points, self.ego, 1.)
        memory.update(points, self.ego, 1.1)
        mover = (8., 0., 1., 'lidar_car', 'npc', 10., 1., 1., 0.,
                 10., 0., 500000, 0., None, True)
        retained = memory.update(np.empty((0, 2)), self.ego, 1.4, [mover])
        self.assertEqual(len(retained), 0)

    def test_moving_pedestrian_clears_nearfield_footprints(self):
        memory = NearFieldMemory()
        points = np.array([[5., -.2], [5., 0.], [5., .2], [5.4, 0.]])
        memory.update(points, self.ego, 1.)
        memory.update(points, self.ego, 1.1)
        mover = (6.4, 0., .6, 'person', 'pedestrian', 2., .6, .6, 0.,
                 2., 0., 500001, 0., None, True)
        retained = memory.update(np.empty((0, 2)), self.ego, 1.4, [mover])
        self.assertEqual(len(retained), 0)

    def test_velocity_uses_world_displacement_and_stable_id(self):
        tracks=VehicleTracks(); ids=[]
        for i in range(9):
            result=tracks.update([dict(position=[20.+15.*i/15.,3.5],size=[4.9,2.1,1.5],heading_deg=0.,score=.9)],10.+i/15.)
            ids.append(result[0]['uid'])
        self.assertEqual(len(set(ids)),1)
        self.assertTrue(result[0]['velocity_valid'])
        self.assertAlmostEqual(result[0]['velocity'][0],15.,places=5)

    def test_delayed_ego_pose_uses_buffered_scan_without_resetting_tracks(self):
        def cloud(stamp):
            return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp)))
        scans = [cloud(10.), cloud(10.125), cloud(10.25)]
        poses = [(10., 0., 0., 0.), (10.1, 2.5, 0., 0.), (10.2, 5., 0., 0.)]
        message, pose = aligned_scan(scans, poses, 10.3, 10.)
        self.assertAlmostEqual(message.header.stamp.to_sec(), 10.25)
        self.assertAlmostEqual(pose[1], 6.25)
        # Waiting for a newer pose preserves the valid 8 Hz result.
        self.assertIsNone(aligned_scan(scans, poses, 10.31, 10.25)[0])
        self.assertFalse(tracked_scan_expired(10.31, 10.25, 10.25))
        # A real sensor outage still invalidates it.
        self.assertTrue(tracked_scan_expired(10.8, 10.25, 10.125))

    def test_exact_mission_links_and_section(self):
        cfg=json.loads((Path(__file__).parent/'highway_mission.json').read_text())
        self.assertEqual((cfg['from_idx'],cfg['to_idx'],cfg['kmh']),(1842,3500,120.))
        self.assertTrue(cfg['use_mission_speed'])
        self.assertEqual(cfg['overtake_kmh'],120.)
        self.assertEqual(cfg['start_link'],'A2256W000866')
        self.assertEqual(cfg['entry_lane_change']['source_link'],'A2256W000411')
        self.assertEqual(cfg['end_link'],'A2256W000153')

    def test_real_map_two_lane_changes_finish_without_stop(self):
        from gui import HdMapLanes, smooth_control_route
        root = Path(__file__).parent
        route = np.array([[p['x'], p['y']] for p in
                          json.loads((root/'global_path.json').read_text())['waypoints']])
        reference = smooth_control_route(route)
        lanes = HdMapLanes(root/'link_set.json')
        lanes.set_route(route)
        config = json.loads((root/'highway_mission.json').read_text())
        # Keep this legacy-policy regression independent of the production
        # three-lane mission's earlier entry and 120 km/h ceiling.
        config.update(from_idx=2251, kmh=90., overtake_kmh=90.,
                      start_link='A2256W000411', lane_change_goal=2,
                      lane_change_policy='adjacent_then_return')
        planner = HighwayPlanner(config, route)
        index = planner.start
        position = reference[index].copy()
        heading = math.atan2(*(reference[index+3]-reference[index])[::-1])
        speed = 25.
        for _ in range(500):
            lo, hi = max(planner.start, index-12), min(planner.end+1, index+32)
            index = lo+int(np.argmin(np.linalg.norm(reference[lo:hi]-position, axis=1)))
            ego = dict(x=float(position[0]), y=float(position[1]),
                       heading_deg=math.degrees(heading), speed_kmh=speed*3.6)
            local = local_points(reference[index:index+640], ego)
            corridor = lanes.corridor(ego, local)
            result = planner.decide(index, local, ego, [], lanes.adjacent_offsets(ego, local),
                                    corridor.clear_distance, lambda path, horizon: horizon)
            self.assertFalse(result['emergency'], (index, result['status']))
            self.assertGreater(result['target_mps'], 1., (index, result['status']))
            self.assertLessEqual(result['target_mps'], 25.)
            if index >= planner.end-1:
                break
            speed += float(np.clip(result['target_mps']-speed, -.6, .24))
            advance = min(5., speed*.2)
            path = result['path']
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
            point = np.array([np.interp(advance, arc, path[:, k]) for k in (0, 1)])
            tangent = np.array([np.interp(advance+.5, arc, path[:, k]) for k in (0, 1)])-point
            position = world_points(point[None, :], ego)[0]
            heading += math.atan2(tangent[1], tangent[0])
        self.assertGreaterEqual(index, planner.end-1)
        self.assertEqual(planner.completed, 2)
        self.assertEqual(planner.phase, 'DONE')


if __name__ == '__main__':
    unittest.main()
