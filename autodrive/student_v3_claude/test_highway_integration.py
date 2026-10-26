"""Highway control integration: model STOP, sensor freshness and collision veto."""
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from gui import Runtime,Image,HdMapLanes,RosPathPublisher,smooth_control_route,highway_nudge
from highway_planner import HighwayPlanner,NearFieldMemory,local_points,world_points
from highway_observations import HighwayObservationFilter
from lidar_corridor import LidarCorridor


class Message:
    def __init__(self,data=None):self.data=data
class Sink:
    def __init__(self):self.items=[]
    def publish(self,message):self.items.append(message)
class Pose:
    def __init__(self):self.header=None;self.pose=SimpleNamespace(position=SimpleNamespace(x=0.,y=0.))
class PathMessage:
    def __init__(self):self.header=SimpleNamespace(stamp=None,frame_id=None);self.poses=[]


class HighwayIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assets=Path(__file__).parent
        cls.world=np.array([[p['x'],p['y']] for p in json.loads((cls.assets/'global_path.json').read_text())['waypoints']])
        cls.smooth=smooth_control_route(cls.world)
        cls.lanes=HdMapLanes(cls.assets/'link_set.json');cls.lanes.set_route(cls.world)

    def test_highway_dispatch_and_state_are_confined_to_mission_indices(self):
        from test_avoidance_gate import AvoidanceGateTests
        mission = json.loads((self.assets/'highway_mission.json').read_text())
        start, end = int(mission['from_idx']), int(mission['to_idx'])
        for index in (start-1, start, end, end+1):
            with self.subTest(index=index):
                runtime = AvoidanceGateTests().runtime(action='KEEP')
                runtime.highway = HighwayPlanner(self.assets/'highway_mission.json', self.world)
                runtime.highway.phase = 'DONE'
                runtime.highway.longitudinal.begin(runtime.test_status['ego'])
                runtime.highway._speed_hold_goal = 10.
                runtime.test_status['path']['nearest_idx'] = index
                with patch.object(runtime, 'infer_highway', return_value=True) as branch:
                    self.assertTrue(runtime.infer())
                if start <= index <= end:
                    branch.assert_called_once()
                else:
                    branch.assert_not_called()
                    self.assertEqual(runtime.highway.phase, 'OUTSIDE')
                    self.assertIsNone(runtime.highway.longitudinal.previous)
                    self.assertIsNone(runtime.highway._speed_hold_goal)
                    self.assertEqual(runtime._web_record['mode'], 'ROUTE BASE')

    def test_highway_dispatch_does_not_read_normal_camera_or_run_student_model(self):
        from unittest.mock import Mock
        from test_avoidance_gate import AvoidanceGateTests
        runtime = AvoidanceGateTests().runtime(action='YIELD')
        runtime.highway = HighwayPlanner(self.assets/'highway_mission.json', self.world)
        runtime.test_status['path']['nearest_idx'] = 2400
        runtime.receiver.snapshot = Mock(side_effect=AssertionError('normal camera gate'))
        runtime.image_tensor = Mock(side_effect=AssertionError('normal camera tensor'))
        runtime.model = Mock(side_effect=AssertionError('normal Student model'))
        with patch.object(runtime, 'infer_highway', return_value=True) as branch:
            self.assertTrue(runtime.infer())
        branch.assert_called_once()
        self.assertEqual(len(branch.call_args.args[2]), 640)
        runtime.receiver.snapshot.assert_not_called()
        runtime.image_tensor.assert_not_called()
        runtime.model.assert_not_called()

    def test_highway_dispatch_still_stops_for_stale_ego_pose(self):
        from test_avoidance_gate import AvoidanceGateTests
        runtime = AvoidanceGateTests().runtime()
        runtime.highway = HighwayPlanner(self.assets/'highway_mission.json', self.world)
        runtime.test_status['path']['nearest_idx'] = 2400
        runtime.test_status['ego']['age_s'] = .5
        with patch.object(runtime, 'infer_highway') as branch:
            self.assertFalse(runtime.infer())
        branch.assert_not_called()
        self.assertEqual(runtime.path_publisher.velocity_pub.items[-1].data, 0.)

    def test_recorded_converging_lane_keeps_occupied_corridor(self):
        frames = json.loads((self.assets/'test_data/highway_corridor_jerk_20261007.json').read_text())
        self.assertTrue(any(frame['original_clear_m'] == .5 for frame in frames))
        for frame in frames:
            ego, index = frame['ego'], frame['index']
            route = local_points(self.smooth[index:index+640], ego)
            path = local_points(np.asarray(frame['path_world']), ego)
            corridor = self.lanes.corridor(ego, route)
            self.assertGreater(corridor.clear_distance(path, 170.), 100., frame['t'])
            # A real operator no-go zone still rejects this same path.
            corridor.zones = [np.array([[-10., -10.], [10., -10.], [10., 10.], [-10., 10.]])]
            self.assertLess(corridor.clear_distance(path, 170.), 1.)

    def run_branch(self,ready=True,collision=False,object_stale=False,objects=None,
                   points=None,source_frames=0,speed_kmh=60.,object_age=None,
                   highway_policy='adjacent_then_return',index=2611,
                   ego_stamp=None,fusion_stamp=None):
        runtime=Runtime.__new__(Runtime)
        mission=json.loads((self.assets/'highway_mission.json').read_text())
        mission['lane_change_policy']=highway_policy
        runtime.highway=HighwayPlanner(mission,self.world)
        runtime.vehicle_receiver=SimpleNamespace(snapshot=lambda:([],ready,dict(ready=ready,vehicles=[])))
        runtime.object_receiver=SimpleNamespace(snapshot=lambda:[],age_s=.6 if object_stale else .01)
        runtime.near_field_memory=NearFieldMemory()
        runtime.lidar=LidarCorridor();runtime.lidar.received=time.monotonic()
        runtime.lanes=self.lanes
        runtime.receiver=SimpleNamespace(rates=lambda:dict(front=30.,left=30.,right=30.))
        runtime.actions=['DRIVE','STOP'];runtime.gallery_matrix=np.empty((0,2));runtime.gallery=[]
        publisher=RosPathPublisher.__new__(RosPathPublisher)
        publisher.Float32=publisher.String=Message;publisher.PoseStamped=Pose;publisher.PathMessage=PathMessage
        publisher.rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda:1.))
        publisher.waypoints_pub=Sink();publisher.velocity_pub=Sink();publisher.mode_pub=Sink();publisher.path_pub=Sink();publisher.status_pub=Sink()
        publisher.cruise_mps=25.;publisher.collision_stop=collision
        publisher.publish_camera_rates=lambda rates:None;publisher.publish_corridor=lambda ego,c:None
        runtime.path_publisher=publisher
        idx=index;x,y=self.world[idx];yaw=math.degrees(math.atan2(*(self.smooth[idx+3]-self.smooth[idx])[::-1]))
        ego=dict(x=x,y=y,heading_deg=yaw,speed_kmh=speed_kmh,age_s=.01)
        if ego_stamp is not None:
            ego['stamp'] = ego_stamp
        observations=objects(ego) if callable(objects) else list(objects or [])
        runtime.object_receiver.snapshot=lambda:observations
        if fusion_stamp is not None:
            runtime.object_receiver.observation_stamp = fusion_stamp
        if object_age is not None:
            runtime.object_receiver.age_s=object_age
        if points is not None:
            runtime.lidar.pts=np.asarray(points,dtype=float).reshape(-1,2)
        if source_frames:
            runtime.object_receiver.observation_stamp=1.
            runtime._highway_observations=HighwayObservationFilter()
            for i in range(source_frames-1):
                runtime._highway_observations.update(observations,1.-(source_frames-1-i)*.1,
                                                     runtime.lidar.pts,ego)
        status=dict(ego=ego,path=dict(nearest_idx=idx,distance_m=0.))
        dense=local_points(self.smooth[idx:idx+640],ego)
        prediction=np.column_stack((np.arange(1.,21.),np.zeros(20)))
        with tempfile.TemporaryDirectory() as directory,patch.dict('os.environ',{'STUDENT_LOG_DIR':directory}):
            runtime.args=SimpleNamespace(status=Path(directory)/'status.json')
            self.assertTrue(runtime.infer_highway(status,ego,dense,dense[:20],dense[::10],prediction,
                np.array([.01,.99]),np.empty(0),np.empty(0),3.,time.perf_counter()))
            runtime._decision_log.close()
        return runtime,publisher

    def test_three_lane_runtime_publishes_highway_mode_and_same_speed_reference(self):
        runtime,publisher=self.run_branch(highway_policy='free_three_lane')
        record=runtime._web_record
        self.assertEqual(record['highway']['policy'],'free_three_lane')
        self.assertEqual(len(record['highway']['available_lanes']),3)
        self.assertEqual(publisher.mode_pub.items[-1].data,'HIGHWAY')
        self.assertAlmostEqual(record['published_cap_kmh'],record['planner_cap_kmh'],places=1)
        self.assertLessEqual(record['published_cap_kmh'],90.)
        self.assertEqual(record['highway']['comfort_decel_mps2'],2.5)

    def test_three_lane_runtime_preserves_physical_collision_and_stale_sensor_veto(self):
        for kwargs in ({'points':[[2.,0.]]},{'object_stale':True,'object_age':2.}):
            with self.subTest(kwargs=kwargs):
                runtime,publisher=self.run_branch(highway_policy='free_three_lane',**kwargs)
                self.assertEqual(publisher.velocity_pub.items[-1].data,0.)

    def test_highway_plan_keeps_the_captured_scan_stamp_during_next_scan(self):
        original = LidarCorridor.xy_snapshot
        def next_scan(receiver):
            result = dict(original(receiver), stamp=123.)
            receiver._scan_stamp = 124.
            return result
        with patch.object(LidarCorridor, 'xy_snapshot', next_scan):
            runtime, _ = self.run_branch(highway_policy='free_three_lane')
        decision = runtime.last_result['postprocess']
        self.assertEqual(decision['sensor_stamp'], 123.)
        self.assertEqual(decision['validation']['sensor_times']['lidar'], 123.)
        self.assertEqual(runtime.lidar._scan_stamp, 124.)

    def test_fusion_npc_is_aligned_once_before_scan_mask_and_collision_forecast(self):
        def npc(ego):
            x, y = world_points(np.array([[30., 3.6]]), ego)[0]
            h = math.radians(ego['heading_deg'])
            return [(x, y, 4.6, 'fusion_npc', 'npc', 11.916, 4.6, 1.9,
                     ego['heading_deg'], 11.916*math.cos(h), 11.916*math.sin(h), 56)]
        runtime, _ = self.run_branch(highway_policy='free_three_lane', objects=npc,
                                    ego_stamp=100.182, fusion_stamp=100., object_age=.182)
        raw = runtime._web_record['fused_observations'][0]
        planned = runtime._web_record['object_tracks'][0]
        np.testing.assert_allclose(planned[:2], np.asarray(raw[:2])+.182*np.asarray(raw[9:11]))
        self.assertEqual(runtime._web_record['highway']['sensor_alignment']['fusion']['aligned_count'], 1)

    def test_timestamped_scan_without_its_pose_cannot_validate_or_fill_memory(self):
        original = LidarCorridor.xy_snapshot
        def timestamped(receiver):
            return dict(original(receiver), stamp=100.)
        with patch.object(LidarCorridor, 'xy_snapshot', timestamped):
            runtime, publisher = self.run_branch(highway_policy='free_three_lane',
                ego_stamp=100.1, points=[[4., 0.], [4.4, .2], [4.8, -.2], [5.2, .4]])
        self.assertEqual(publisher.velocity_pub.items[-1].data, 0.)
        self.assertFalse(runtime.last_result['postprocess']['validation']['valid'])
        self.assertEqual(runtime._web_record['reason'], 'highway scan pose unavailable')
        self.assertFalse(runtime.near_field_memory.cells)

    def test_three_lane_runtime_old_contact_does_not_latch_a_stop(self):
        runtime,publisher=self.run_branch(highway_policy='free_three_lane',collision=True)
        self.assertGreater(publisher.velocity_pub.items[-1].data,0.)
        self.assertFalse(runtime._web_record['emergency_stop'])

    def test_three_lane_exit_command_keeps_sixty_kmh_boundary_despite_momentum(self):
        runtime,publisher=self.run_branch(highway_policy='free_three_lane',index=3500,speed_kmh=72.)
        self.assertEqual(publisher.mode_pub.items[-1].data,'HIGHWAY')
        self.assertLessEqual(publisher.velocity_pub.items[-1].data*3.6,60.+1e-6)
        self.assertTrue(runtime._web_record['highway']['speed_plan']['reference_checked'])

    def test_overlapping_camera_history_reuses_decode_with_bounded_cache(self):
        import io
        runtime=Runtime.__new__(Runtime)
        runtime.config=dict(width=64,height=32)
        def jpeg(color):
            stream=io.BytesIO();Image.new('RGB',(64,32),color).save(stream,format='JPEG')
            return stream.getvalue()
        frames=[jpeg((i*12,0,0)) for i in range(15)]
        rows=[frames[i:i+3] for i in range(0,12,3)]
        with patch('gui.Image.open',wraps=Image.open) as opened:
            first,_=runtime.image_tensor(rows)
            second,_=runtime.image_tensor(rows)
            self.assertEqual(opened.call_count,12)
            self.assertTrue(np.array_equal(first.numpy(),second.numpy()))
            runtime.image_tensor(rows[1:]+[frames[12:15]])
            self.assertEqual(opened.call_count,15)
            self.assertEqual(len(runtime._image_cache),15)
            for i in range(60):
                runtime.image_tensor([[jpeg((0,i,0))]*3]*4)
            self.assertLessEqual(len(runtime._image_cache),64)

    def test_raw_model_stop_does_not_stop_clear_highway(self):
        runtime,publisher=self.run_branch()
        self.assertEqual(runtime._web_record['action'],'STOP')
        self.assertEqual(runtime._web_record['motion_state'],'DRIVE')
        self.assertGreater(publisher.velocity_pub.items[-1].data,60./3.6)
        from highway_speed_plan import (FOLLOWER_ACCEL_GAIN, FOLLOWER_FULL_ACCEL_MPS2,
                                        HIGHWAY_ACCEL_MPS2)
        self.assertLessEqual(publisher.velocity_pub.items[-1].data,
            60./3.6 + HIGHWAY_ACCEL_MPS2 / (FOLLOWER_ACCEL_GAIN * FOLLOWER_FULL_ACCEL_MPS2))
        self.assertTrue(runtime._web_record['highway']['speed_plan']['reference_checked'])
        payload=json.loads(publisher.waypoints_pub.items[-1].data)
        self.assertGreater(len(payload['waypoints']),300)
        self.assertTrue(runtime.last_result['postprocess']['validated_static'])
        self.assertEqual(publisher.mode_pub.items[-1].data,'HIGHWAY')

    def test_separate_highway_branch_publishes_without_model_outputs(self):
        from unittest.mock import Mock
        runtime,publisher=self.run_branch()
        runtime.input_error=None
        runtime.update_ego=lambda:runtime.last_result['status']
        runtime.receiver.snapshot=Mock(side_effect=AssertionError('normal camera input'))
        runtime.model=Mock(side_effect=AssertionError('normal model'))
        self.assertTrue(runtime.infer())
        self.assertEqual(runtime._web_record['mode'],'HIGHWAY')
        self.assertEqual(runtime._web_record['action'],'HIGHWAY')
        self.assertEqual(len(runtime.last_result['probabilities']),0)
        self.assertGreater(publisher.velocity_pub.items[-1].data,60./3.6)
        runtime.receiver.snapshot.assert_not_called()
        runtime.model.assert_not_called()

    def test_final_highway_path_owns_curvature_limit_before_publication(self):
        with patch('highway_runtime.geometry_speed_cap', return_value=35./3.6):
            runtime, publisher = self.run_branch()
        info = runtime._web_record['highway']
        self.assertEqual(info['final_path_curve_cap_kmh'], 35.)
        self.assertLessEqual(info['speed_plan']['requested_goal_kmh'], 35.)
        self.assertLess(publisher.velocity_pub.items[-1].data, 60./3.6)
        self.assertEqual(publisher.mode_pub.items[-1].data, 'HIGHWAY')

    def test_sensor_stale_or_model_not_ready_stops(self):
        for kwargs in (dict(ready=False),dict(object_stale=True)):
            with self.subTest(kwargs=kwargs):
                runtime,publisher=self.run_branch(**kwargs)
                self.assertEqual(publisher.velocity_pub.items[-1].data,0.)
                self.assertTrue(runtime._web_record['emergency_stop'])

    def test_old_collision_notification_does_not_stop_clear_highway(self):
        runtime,publisher=self.run_branch(collision=True)
        self.assertGreater(publisher.velocity_pub.items[-1].data,60./3.6)
        self.assertFalse(runtime._web_record['emergency_stop'])
        self.assertIsNone(runtime._web_record['stop_reason'])

    @staticmethod
    def close_obstacle(ego,name='fusion_obstacle',kind='obstacle'):
        x,y=world_points(np.array([[5.,0.]]),ego)[0]
        return [(x,y,1.1,name,kind,0.,1.1,1.1,ego['heading_deg'],0.,0.,1)]

    def test_single_unsupported_semantic_frame_slows_without_emergency_stop(self):
        runtime,publisher=self.run_branch(objects=self.close_obstacle,source_frames=1)
        self.assertFalse(runtime._web_record['emergency_stop'])
        self.assertLess(publisher.velocity_pub.items[-1].data,60./3.6)
        self.assertGreaterEqual(publisher.velocity_pub.items[-1].data,60./3.6-.4)
        self.assertEqual(runtime._web_record['highway']['observations']['pending'],1)

    def test_one_raw_collision_return_still_stops_on_first_frame(self):
        runtime,publisher=self.run_branch(objects=self.close_obstacle,source_frames=1,points=[[5.,0.]])
        self.assertTrue(runtime._web_record['emergency_stop'])
        self.assertEqual(publisher.velocity_pub.items[-1].data,0.)

    def test_confirmed_semantic_obstacle_stops_without_current_lidar_support(self):
        runtime,publisher=self.run_branch(objects=self.close_obstacle,source_frames=3)
        self.assertTrue(runtime._web_record['emergency_stop'])
        self.assertEqual(publisher.velocity_pub.items[-1].data,0.)
        self.assertEqual(runtime._web_record['highway']['observations']['pending'],0)

    def test_low_speed_obstacle_and_pedestrian_keep_first_frame_stop(self):
        for kwargs in (dict(speed_kmh=3.,objects=self.close_obstacle),
                       dict(objects=lambda ego:self.close_obstacle(ego,'fusion_pedestrian','pedestrian'))):
            with self.subTest(kwargs=kwargs):
                runtime,publisher=self.run_branch(source_frames=1,**kwargs)
                self.assertEqual(publisher.velocity_pub.items[-1].data,0.)

    def test_capture_staleness_stops_even_if_receipt_is_recent(self):
        runtime,publisher=self.run_branch(source_frames=1,object_age=1.2)
        self.assertTrue(runtime._web_record['emergency_stop'])
        self.assertEqual(publisher.velocity_pub.items[-1].data,0.)
        self.assertTrue(runtime._web_record['object_stale'])

    def test_roadside_nudge_keeps_path_under_vehicle_when_scan_changes(self):
        from scipy.spatial import cKDTree
        path=np.column_stack((np.arange(0.,100.5,.5),np.zeros(201)))
        points=np.column_stack((np.arange(0.,90.,.5),np.full(180,-1.4)))
        nudged=highway_nudge(path,points,cKDTree(points),25.)
        absent=highway_nudge(path,np.empty((0,2)),None,25.)
        np.testing.assert_array_equal(nudged[0],absent[0])
        self.assertLess(abs(nudged[2,1]-absent[2,1]),.001)
        self.assertGreater(nudged[100,1],.4)
        self.assertLessEqual(np.max(abs(nudged[:,1])),.6)

    def test_nudge_cannot_hide_a_collision_inside_current_body(self):
        from scipy.spatial import cKDTree
        from tunnel_runtime import swept_clearance
        path=np.column_stack((np.arange(0.,100.5,.5),np.zeros(201)))
        points=np.array([[1.,.8],[1.1,.8]])
        nudged=highway_nudge(path,points,cKDTree(points),25.)
        self.assertEqual(swept_clearance(nudged,points,100.,extra_margin=.25,lateral_margin=.15),0.)


if __name__=='__main__':unittest.main()
