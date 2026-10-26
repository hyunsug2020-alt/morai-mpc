"""Production entrypoint and measured-pose frame contract on ordinary roads."""
import ast
import inspect
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import semantic_fusion as fusion

HERE=Path(__file__).resolve().parent
LAUNCHER=HERE.parent/'student_v3_follow_claude.sh'


class FusionEntrypointTests(unittest.TestCase):
    def test_fresh_cloud_selection_preserves_source_deadlines_and_camera_alignment(self):
        old, fresh = (10.02, np.zeros((1, 3))), (10.18, np.ones((1, 3)))
        chosen = fusion.select_semantic_cloud([old, fresh], 10.1, 10.1,
            [10.1]*3, 10.3, processing_reserve_s=1./15.)
        self.assertIs(chosen, fresh)
        self.assertTrue(fusion.semantic_source_timing(chosen[0], 10.1, [10.1]*3, 10.3)['ready'])
        self.assertFalse(fusion.semantic_source_timing(chosen[0], 10.1, [10.1]*3, 10.6)['ready'])
        # A sufficiently fresh closest frame still wins over a newer but
        # more distant camera match. Repeated physical frames never win.
        near = (10.12, np.zeros((1, 3)))
        self.assertIs(fusion.select_semantic_cloud([near, fresh], 10.1, 10.1,
            [10.1]*3, 10.2, processing_reserve_s=1./15.), near)
        self.assertIs(fusion.select_semantic_cloud([near, fresh], 10.1, 10.1,
            [10.1]*3, 10.2, last_cloud_stamp=10.12), fresh)
        rejected = fusion.select_semantic_cloud([old, fresh], 10.1, 10.1,
            [9.8, 10.1, 10.1], 10.3)
        self.assertFalse(fusion.semantic_source_timing(rejected[0], 10.1,
            [9.8, 10.1, 10.1], 10.3)['ready'])

    def launch(self, root, override=None, missing=False):
        source=LAUNCHER.read_text()
        assignment=re.search(r'^ASSETS=.*$',source,re.M).group(0)
        function=re.search(r'^start_semantic_fusion\(\) \{\n.*?^\}\n',source,re.M|re.S).group(0)
        selected=Path(override) if override is not None else Path(root)/'autodrive/student_v3_claude'
        selected.mkdir(parents=True)
        if not missing:
            (selected/'semantic_fusion.py').write_text(
                'import json,sys\nprint(json.dumps({"entrypoint":__file__,"args":sys.argv[1:]}))\n')
        env=dict(os.environ,ROOT=str(root),PYTHON=sys.executable,SENSOR_PRESET='sensor file.json')
        env.pop('STUDENT_ASSETS',None)
        if override is not None:env['STUDENT_ASSETS']=str(override)
        result=subprocess.run(['bash','-c',assignment+'\n'+function+'\nstart_semantic_fusion'],
                              env=env,capture_output=True,text=True)
        return result,selected

    def test_production_and_staging_commands_execute_the_exact_assets_script(self):
        for staging in (False,True):
            with self.subTest(staging=staging),tempfile.TemporaryDirectory() as root:
                result,assets=self.launch(root,Path(root)/'staging copy' if staging else None)
                self.assertEqual(result.returncode,0,result.stderr)
                record=json.loads(result.stdout.splitlines()[-1])
                self.assertEqual(record['entrypoint'],str(assets/'semantic_fusion.py'))
                self.assertEqual(record['args'],['--sensor','sensor file.json','--route',str(assets/'global_path.json')])
                self.assertIn(str(assets/'semantic_fusion.py'),result.stdout.splitlines()[0])

    def test_missing_assets_file_never_falls_back_to_obsolete_root_copy(self):
        with tempfile.TemporaryDirectory() as root:
            obsolete=Path(root)/'autodrive/semantic_fusion.py'
            obsolete.parent.mkdir(parents=True)
            obsolete.write_text('raise AssertionError("obsolete fusion executed")\n')
            result,_=self.launch(root,Path(root)/'empty staging',missing=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('semantic fusion entrypoint missing',result.stderr)
            self.assertNotIn('obsolete fusion executed',result.stderr)

    def test_launcher_calls_the_checked_function_before_external_setup(self):
        source=LAUNCHER.read_text()
        self.assertIn('start_semantic_fusion >"$RUN_DIR/semantic_fusion.log"',source)
        self.assertNotIn('"$PYTHON" "$ROOT/autodrive/semantic_fusion.py"',source)
        self.assertLess(source.index('[[ -s "$ASSETS/semantic_fusion.py" ]]'),source.index('PROFILE_ENV='))
        self.assertEqual(Path(fusion.__file__).resolve(),HERE/'semantic_fusion.py')
        subprocess.run(['bash','-n',str(LAUNCHER)],check=True,capture_output=True)


class FusionAlignmentTests(unittest.TestCase):
    @staticmethod
    def world(points,pose):
        c,s=math.cos(pose[3]),math.sin(pose[3])
        return points[:,:2]@np.array([[c,s],[-s,c]])+pose[1:3]

    def test_static_world_point_is_fixed_despite_changing_rtf_and_native_velocity(self):
        history=[[10.,0.,0.,0.,20.,0.],[10.1,.4,0.,0.,20.,0.],
                 [10.2,1.8,0.,0.,20.,0.]]
        world=np.array([[8.,1.,.4],[12.,-2.,1.5]],np.float32)
        for source,target in ((history[0],history[1]),(history[1],history[2])):
            points=world.copy();points[:,:2]-=source[1:3]
            actual,info=fusion.align_semantic_cloud(points,source[0],target,history,target[0])
            np.testing.assert_allclose(self.world(actual,target),world[:,:2],atol=1e-6)
            self.assertEqual(info['mode'],'measured_pose')
            self.assertFalse(info['legacy_fallback'])
            self.assertEqual(len(actual),len(points))
            self.assertEqual(actual.dtype,points.dtype)
        # Actual first motion=.4m; multiplying native20m/s bywall.1s would shift2m.

    def test_camera_nearest_ego_stamp_and_rotation_are_the_exact_publication_frame(self):
        source=[10.,2.,3.,math.pi/2.,2.,0.]
        target=[10.08,2.1,3.,math.pi/2.+.08,2.,0.]
        points=np.array([[1.,2.,.4],[3.,-1.,1.5]],np.float64)
        original=points.copy()
        actual,info=fusion.align_semantic_cloud(points,10.,target,[source,target],10.103)
        np.testing.assert_allclose(self.world(actual,target),self.world(points,source),atol=1e-12)
        np.testing.assert_array_equal(actual[:,2],points[:,2])
        np.testing.assert_array_equal(points,original)
        self.assertAlmostEqual(info['target_lidar_dt_ms'],80.)
        self.assertAlmostEqual(info['camera_target_dt_ms'],23.)

    def test_missing_pose_is_explicit_fallback_without_return_deletion_permission(self):
        points=np.array([[5.,1.,.4],[8.,-2.,1.5]],np.float32)
        actual,info=fusion.align_semantic_cloud(points,10.,[10.1,.1,0.,0.,2.,0.],[],10.15)
        np.testing.assert_allclose(actual[:,:2],points[:,:2]-[.2,0.],atol=1e-6)
        self.assertTrue(info['legacy_fallback'])
        self.assertFalse(info['source_pose_valid'])
        self.assertFalse(info['infrastructure_aligned'])
        self.assertEqual(len(actual),len(points))
        self.assertEqual(info['oldest_source_stamp'],10.)

    def test_stale_cloud_is_unaligned_and_not_refreshed_by_a_new_camera(self):
        points=np.array([[5.,1.,.4],[8.,-2.,1.5]],np.float32)
        actual,info=fusion.align_semantic_cloud(points,10.,[10.5,1.,0.,0.,2.,0.],[],10.5)
        np.testing.assert_array_equal(actual,points)
        self.assertEqual(info['oldest_source_stamp'],10.)
        self.assertEqual(info['mode'],'unaligned')
        self.assertFalse(info['infrastructure_aligned'])
        self.assertFalse(info['source_pose_valid'])
        self.assertGreater(10.5-info['oldest_source_stamp'],.3)

    def test_oldest_evidence_is_a_gate_and_not_the_dynamic_motion_timestamp(self):
        from highway_sensor_alignment import align_moving_observations
        source=[10.2,2.,0.,0.,10.,0.]
        target=[10.1,1.,0.,0.,10.,0.]
        points=np.array([[8.,0.,.5]])  # moving body's observed world position=10 at cloud10.2
        aligned,_=fusion.align_semantic_cloud(points,10.2,target,[target,source],10.18)
        position=self.world(aligned,target)[0]
        timing=fusion.semantic_source_timing(10.2,10.1,[10.18]*3,10.3)
        self.assertTrue(timing['ready'])
        self.assertEqual(timing['motion_stamp'],10.2)
        self.assertEqual(timing['oldest_source_stamp'],10.1)
        for kind,velocity in (('npc',10.),('pedestrian',1.)):
            row=(*position,1.,'fusion_'+kind,kind,velocity,1.,1.,0.,velocity,0.,1)
            forecast,_=align_moving_observations([row],timing['motion_stamp'],10.3)
            self.assertAlmostEqual(forecast[0][0],10.+velocity*.1)
            wrong,_=align_moving_observations([row],timing['oldest_source_stamp'],10.3)
            self.assertAlmostEqual(wrong[0][0]-forecast[0][0],velocity*.1)

    def test_stale_camera_or_pose_proof_prevents_a_fresh_cloud_publication(self):
        for target,cameras in ((10.5,[10.,10.5,10.5]),(10.,[10.5]*3)):
            with self.subTest(target=target,cameras=cameras):
                timing=fusion.semantic_source_timing(10.5,target,cameras,10.5)
                self.assertFalse(timing['ready'])
                self.assertEqual(timing['motion_stamp'],10.5)
                self.assertEqual(timing['oldest_source_stamp'],10.)

    def test_duplicate_cloud_is_rejected_and_clock_reset_starts_a_new_epoch(self):
        timing=fusion.semantic_source_timing(10.2,10.2,[10.2]*3,10.3,10.2)
        self.assertFalse(timing['ready'])
        self.assertIn('repeated',timing['reason'])
        reset=fusion.semantic_source_timing(2.,2.,[2.]*3,2.02,10.2)
        self.assertTrue(reset['ready'])
        self.assertTrue(reset['source_clock_reset'])
        repeated=fusion.semantic_source_timing(2.,2.,[2.]*3,2.03,2.)
        self.assertFalse(repeated['ready'])

    def test_future_source_bound_matches_the_existing_receiver_clock_contract(self):
        self.assertFalse(fusion.semantic_source_timing(10.36,10.3,[10.3]*3,10.3)['ready'])
        self.assertTrue(fusion.semantic_source_timing(10.34,10.3,[10.3]*3,10.3)['ready'])
        self.assertFalse(fusion.semantic_source_timing(float('nan'),10.3,[10.3]*3,10.3)['ready'])

    def test_rejected_sources_replace_old_diagnostics_without_fresh_object_publication(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'fusion.json'
            path.write_text('{"time":1,"source_timing_valid":true}')
            timing=fusion.semantic_source_timing(10.5,10.5,[10.,10.5,10.5],10.5)
            with patch.object(fusion.time,'time',return_value=20.):
                fusion.write_source_rejection(path,timing,{'stale':1},0.,15.)
            record=json.loads(path.read_text())
            self.assertEqual(record['time'],20.)
            self.assertFalse(record['source_timing_valid'])
            self.assertEqual(record['sensor_alignment'],'source_rejected')
            self.assertEqual(record['alignment']['source_timing']['oldest_source_stamp'],10.)
            self.assertFalse((Path(directory)/'.fusion.json.tmp').exists())

    def test_processing_delay_rechecks_oldest_evidence_before_publication(self):
        initial=fusion.semantic_source_timing(10.2,10.18,[10.18]*3,10.3)
        final=fusion.semantic_source_timing(10.2,10.18,[10.18]*3,10.6)
        self.assertTrue(initial['ready'])
        self.assertFalse(final['ready'])
        source=inspect.getsource(fusion.main)
        self.assertLess(source.index("if not final_timing['ready']:"),source.index('pub.publish(out)'))

    def test_nonjunction_alignment_and_physical_source_clock_are_wired_into_main(self):
        source=inspect.getsource(fusion.main)
        self.assertNotIn('if shape_active else None',source)
        calls=[n for n in ast.walk(ast.parse(source)) if isinstance(n,ast.Call) and
               isinstance(n.func,ast.Name) and n.func.id=='align_semantic_cloud']
        self.assertEqual(len(calls),1)
        self.assertIn('kind == "obstacle" and shape_active and infrastructure_aligned',source)
        self.assertIn("kind == 'pedestrian'",inspect.getsource(fusion.filter_infrastructure_points))
        self.assertIn("rospy.Time.from_sec(source_timing['motion_stamp'])",source)
        self.assertIn("tracker.update(kind, dets, source_timing['motion_stamp'])",source)
        self.assertIn("tracker.update(kind, [], source_timing['motion_stamp'])",source)
        self.assertNotIn('tracker.update(kind, [], cam_stamp)',source)
        self.assertIn("pedestrian_parts.merge(tracked_detections, source_timing['motion_stamp'])",source)
        self.assertIn("camera_stamps = [int(masks[f'{view}_stamp_ns'])*1e-9 for view in VIEWS]",source)
        self.assertLess(source.index("if not source_timing['ready']:"),source.index('align_semantic_cloud('))
        self.assertLess(source.index("if not source_timing['ready']:"),source.index('tracker.update('))
        self.assertIn('[semantic_fusion] entrypoint=',source)


if __name__=='__main__':unittest.main()
