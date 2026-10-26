"""Tunnel-only exit returns, completion deadlines and preparation speed."""
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import gui
import tunnel_runtime as tr
from ego_body import REAR_AXLE_SHIFT_M


def route(offset=-3., length=100.):
    station = np.arange(0., length+.01, .5)
    return np.column_stack((station, np.full(len(station), offset)))


def obstacle(x, y):
    return (x, y, 4.8, 'standing_car', 'npc', 0., 4.8, 2.1, 0., 0., 0., 42)


class ExitRejoinTests(unittest.TestCase):
    def choose(self, remaining=26., offset=-3., speed=10., objects=(), hazards=(), clearance=None):
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=speed)
        self.tunnel = SimpleNamespace(origin=np.zeros(2), axis=np.array([1., 0.]), end=remaining)
        callback = clearance or (lambda p,h: dict(wall=h, lane=h, obstacle=h))
        return tr.tunnel_exit_rejoin(route(offset), self.ego, self.tunnel, objects, hazards,
                                     callback, route(0.), budget_s=.5)

    def test_prefers_return_completed_two_metres_before_portal(self):
        plan, info = self.choose()
        self.assertIsNotNone(plan, info)
        self.assertTrue(info['completes_before_exit'], info)
        self.assertLessEqual(plan['transition_m'], 24.)
        self.assertGreaterEqual(info['completion_to_exit_m'], 1.95)
        self.assertEqual(plan['offset_m'], 0.)

    def test_all_checks_receive_exact_canonical_publication_vertices(self):
        checked = []
        def clear(path, horizon):
            checked.append(path.copy())
            self.assertEqual(path.shape[1], 2)
            self.assertLessEqual(np.linalg.norm(np.diff(path,axis=0),axis=1).max(), .501)
            return dict(wall=horizon, lane=horizon, obstacle=horizon)
        plan, info = self.choose(clearance=clear)
        self.assertTrue(info['selected'])
        np.testing.assert_array_equal(plan['path'], checked[-1])

    def test_unreachable_deadline_uses_earliest_safe_current_speed_return(self):
        plan, info = self.choose(remaining=12., speed=30.)
        self.assertIsNotNone(plan, info)
        self.assertFalse(info['completes_before_exit'])
        self.assertGreater(info['rejected']['dynamics'], 0)
        self.assertLessEqual(30./3.6, tr.curve_speed_cap(plan['path'], 30./3.6)[0]+.5)
        self.assertIn('deadline unreachable', info['reason'])

    def test_raw_wall_and_lane_failures_never_authorize_return(self):
        for source in ('wall','lane','obstacle'):
            with self.subTest(source=source):
                plan, info = self.choose(clearance=lambda p,h:dict(wall=h,lane=h,obstacle=h)|{source:3.})
                self.assertIsNone(plan)
                self.assertGreater(info['rejected']['geometry'], 0)

    def test_static_sat_is_independent_of_raw_return_callback(self):
        plan, info = self.choose(objects=[obstacle(2., 0.)])
        self.assertIsNone(plan)
        self.assertGreater(info['rejected']['geometry'], 0)

    def test_unrelated_hazard_does_not_block_fully_checked_return(self):
        plan, info = self.choose(objects=[obstacle(2., -8.)],
                                 hazards=[dict(front=2.,along=2.5,back=3.,lat=-6.)])
        self.assertIsNotNone(plan, info)
        self.assertTrue(info['completes_before_exit'])

    def test_last_route_car_prepares_speed_before_return_becomes_clear(self):
        car=obstacle(11.4,-3.)
        plan, info = self.choose(remaining=30.,speed=32.,objects=[car],
                                 hazards=[dict(front=9.,along=11.4,back=13.8,lat=-3.)])
        self.assertIsNotNone(info['preparation_cap_kmh'], info)
        self.assertGreater(info['preparation_cap_kmh'], 0.)
        self.assertLess(info['preparation_cap_kmh'], 32.)
        self.assertAlmostEqual(info['preparation_cap_kmh'],
            math.sqrt(tr.TUNNEL_BYPASS_ALAT_MPS2/info['preparation_curvature_1pm'])*3.6)
        if plan is not None:
            self.assertGreaterEqual(gui.static_clear_distance(plan['path'],self.ego,[car],
                plan['safe_horizon_m'],body_shift_m=REAR_AXLE_SHIFT_M),plan['safe_horizon_m']-.3)

    def test_preparation_retains_car_whose_front_is_behind_but_rear_is_alongside(self):
        _, info = self.choose(remaining=20., speed=32., objects=[obstacle(.9, -3.)],
            hazards=[dict(front=-1.5, along=.9, back=3.5, lat=-3.)])
        self.assertGreater(info['preparation_ahead_m'], 3.5)
        self.assertTrue(info['preparation_steerable'])
        self.assertLessEqual(info['preparation_curvature_1pm'], gui.MAX_CONTROL_CURVATURE)

    def test_exit_wall_comfort_margin_remains_an_independent_rejection(self):
        ego=dict(x=0., y=0., heading_deg=0., speed_kmh=10.)
        tunnel=SimpleNamespace(origin=np.zeros(2), axis=np.array([1.,0.]), end=26.)
        plan,info=tr.tunnel_exit_rejoin(route(), ego, tunnel, [], [],
            lambda p,h:dict(wall=h, lane=h, obstacle=h), route(0.),
            wall_clearance=lambda p,h:3., centre_limit=60., budget_s=.5)
        self.assertIsNone(plan)
        self.assertGreater(info['rejected']['geometry'],0)

    def test_alignment_does_not_confuse_selection_with_completion(self):
        self.assertFalse(tr.tunnel_route_alignment(route(-3.))['complete'])
        self.assertTrue(tr.tunnel_route_alignment(route(0.))['complete'])
        angled=route(0.);angled[:,1]=.1*angled[:,0]
        self.assertFalse(tr.tunnel_route_alignment(angled)['complete'])

    def test_budget_failure_keeps_existing_path(self):
        ego=dict(x=0.,y=0.,heading_deg=0.,speed_kmh=10.)
        tunnel=SimpleNamespace(origin=np.zeros(2),axis=np.array([1.,0.]),end=26.)
        plan,info=tr.tunnel_exit_rejoin(route(),ego,tunnel,[],[],
            lambda p,h:(_ for _ in ()).throw(AssertionError('expired search ran checks')),
            route(0.),budget_s=0.)
        self.assertIsNone(plan)
        self.assertTrue(info['budget_exhausted'])

    def test_scope_excludes_every_nonexit_and_outside_tunnel_frame(self):
        for remaining in (-1.,30.1,60.):
            with self.subTest(remaining=remaining):
                plan,info=self.choose(remaining=remaining)
                self.assertIsNone(plan)
                self.assertEqual(info['reason'],'outside tunnel exit zone')
                self.assertIsNone(info['preparation_cap_kmh'])


class ExitIntegrationTests(unittest.TestCase):
    def runtime(self):
        from test_tunnel_integration import TunnelIntegrationTests
        runtime=TunnelIntegrationTests().runtime()
        runtime.tunnel.origin=np.zeros(2)
        runtime.tunnel.axis=np.array([1.,0.])
        runtime.tunnel.start=-100.
        runtime.tunnel.end=26.
        wall=runtime.tunnel.path_clearance
        runtime.tunnel.path_clearance=lambda p,e,h,margin=.25:wall(p,e,h)
        process=runtime.tunnel.process
        runtime.tunnel.process=lambda xyz,e,plane,**kwargs:process(xyz,e,plane)
        runtime.test_route[:,1]=-3.
        return runtime

    def test_preparation_cap_survives_same_cycle_retry(self):
        runtime=self.runtime()
        def retry(decision, dense, corridor, frame, points, *args):
            return decision.decided(allowed_mps=50.),points,None
        runtime.retry_tunnel_plan=retry
        with patch.object(tr,'tunnel_exit_rejoin',return_value=(None,dict(preparation_cap_kmh=7.2))):
            runtime.infer_tunnel(runtime.test_status,runtime.test_status['ego'])
        row=runtime._web_record
        self.assertTrue(row['tunnel']['exit_rejoin']['active'])
        self.assertLessEqual(row['planner_cap_kmh'],7.2+1e-6)
        self.assertLessEqual(row['published_cap_kmh'],7.2+1e-6)

    def test_selected_canonical_return_is_not_resampled_again_before_final_checks(self):
        runtime=self.runtime()
        original=tr.tunnel_exit_rejoin
        selected=[]
        def choose(*args,**kwargs):
            plan,info=original(*args,**kwargs)
            self.assertIsNotNone(plan,info)
            selected.append(plan['path'].copy())
            return plan,info
        checked=[]
        original_check=runtime.checked_tunnel_plan
        def check(path,*args,**kwargs):
            checked.append(path.copy())
            return original_check(path,*args,**kwargs)
        runtime.checked_tunnel_plan=check
        with patch.object(tr,'tunnel_exit_rejoin',side_effect=choose):
            runtime.infer_tunnel(runtime.test_status,runtime.test_status['ego'])
        self.assertTrue(selected)
        np.testing.assert_array_equal(checked[0],selected[0])
        self.assertFalse(runtime._web_record['tunnel']['rejoin_complete'])

    def test_canonical_curvature_boundary_is_rejected(self):
        angle=math.radians(-7.)
        station=np.arange(0.,100.01,.5)
        dense=np.column_stack((station*math.cos(angle),-4.5+station*math.sin(angle)))
        original=gui.build_forced_bypass
        raw,peak,_=original(dense,0.,offset_m=0.,hold_m=0.,
            max_curvature_1pm=gui.MAX_CONTROL_CURVATURE,transitions=(12.,))
        self.assertLess(peak,gui.MAX_CONTROL_CURVATURE)
        self.assertGreater(np.max(gui.path_curvatures(gui.RosPathPublisher.resample(raw))),
                           gui.MAX_CONTROL_CURVATURE)
        # Force the recorded boundary geometry for every nominal transition.
        with patch.object(gui,'build_forced_bypass',return_value=(raw,peak,12.)):
            plan,info=tr.tunnel_exit_rejoin(dense,dict(x=0.,y=0.,heading_deg=0.,speed_kmh=5.),
                SimpleNamespace(origin=np.zeros(2),axis=np.array([1.,0.]),end=26.),[],[],
                lambda p,h:dict(wall=h,lane=h,obstacle=h),route(0.),budget_s=.5)
        self.assertIsNone(plan)
        self.assertGreater(info['rejected']['curvature'],0)

class ExitCommitTests(unittest.TestCase):
    def seed(self):
        fixture=ExitRejoinTests()
        plan,info=fixture.choose(remaining=26.,speed=10.)
        self.assertTrue(info['completes_before_exit'])
        return fixture.ego,fixture.tunnel,plan,dict(stamp=1.,plan=plan,world_path=plan['path'].copy())

    @staticmethod
    def advance(ego,plan,travel=1.):
        path=np.asarray(plan['path'])
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
        local=np.array([np.interp(travel,arc,path[:,k]) for k in (0,1)])
        index=min(int(np.searchsorted(arc,travel)),len(path)-2)
        tangent=path[index+1]-path[index]
        world=local@tr.rotation(ego).T+[ego['x'],ego['y']]
        return dict(ego,x=world[0],y=world[1],heading_deg=ego['heading_deg']+
                    math.degrees(math.atan2(tangent[1],tangent[0])))

    @staticmethod
    def dense(ego):
        local=(route()-[ego['x'],ego['y']])@tr.rotation(ego)
        nearest=int(np.argmin(np.linalg.norm(local,axis=1)))
        return local[max(0,nearest-1):]

    def test_checked_world_return_wins_without_building_fresh_candidates(self):
        ego,tunnel,plan,saved=self.seed()
        ego=self.advance(ego,plan,4.)
        with patch.object(gui,'build_forced_bypass',side_effect=AssertionError('fresh return reset commitment')):
            selected,info=tr.tunnel_exit_rejoin(self.dense(ego),ego,tunnel,[],[],
                lambda p,h:dict(wall=h,lane=h,obstacle=h),previous_saved=saved,now=1.1,budget_s=.5)
        self.assertTrue(info['reused'],info)
        self.assertGreaterEqual(info['completion_to_exit_m'],1.95)
        self.assertEqual(selected['exit_rejoin_goal_xy'],plan['exit_rejoin_goal_xy'])

    def test_repeated_pose_updates_finish_before_portal_without_resetting_target(self):
        ego,tunnel,plan,saved=self.seed()
        goal=plan['exit_rejoin_goal_xy'].copy()
        for step in range(1,28):
            ego=self.advance(ego,plan,.8)
            dense=self.dense(ego)
            alignment=tr.tunnel_route_alignment(dense)
            if alignment['complete']:
                self.assertGreaterEqual(tunnel.end-ego['x'],2.)
                break
            selected,info=tr.tunnel_exit_rejoin(dense,ego,tunnel,[],[],
                lambda p,h:dict(wall=h,lane=h,obstacle=h),previous_saved=saved,
                now=1.+step*.1,budget_s=.5)
            self.assertTrue(info['reused'],info)
            self.assertEqual(selected['exit_rejoin_goal_xy'],goal)
            plan=selected
            saved=dict(stamp=1.+step*.1,plan=plan,
                       world_path=plan['path']@tr.rotation(ego).T+[ego['x'],ego['y']])
        else:
            self.fail('committed return never reached measured alignment')

    def test_new_raw_or_wall_blocker_invalidates_committed_return(self):
        for source in ('wall','lane','obstacle'):
            with self.subTest(source=source):
                ego,tunnel,plan,saved=self.seed()
                ego=self.advance(ego,plan,4.)
                selected,info=tr.tunnel_exit_rejoin(self.dense(ego),ego,tunnel,[],[],
                    lambda p,h:dict(wall=h,lane=h,obstacle=h)|{source:1.},
                    previous_saved=saved,now=1.1,budget_s=.5)
                self.assertIsNone(selected)
                self.assertTrue(info['committed_invalidated'])
                self.assertGreater(info['rejected']['geometry'],0)

    def test_new_static_sat_blocker_invalidates_even_if_raw_callback_is_clear(self):
        ego,tunnel,plan,saved=self.seed()
        ego=self.advance(ego,plan,4.)
        pos=np.array([2.,0.])@tr.rotation(ego).T+[ego['x'],ego['y']]
        selected,info=tr.tunnel_exit_rejoin(self.dense(ego),ego,tunnel,[obstacle(*pos)],[],
            lambda p,h:dict(wall=h,lane=h,obstacle=h),previous_saved=saved,now=1.1,budget_s=.5)
        self.assertIsNone(selected)
        self.assertTrue(info['committed_invalidated'])

    def test_stale_saved_return_falls_back_to_fresh_checked_search(self):
        ego,tunnel,plan,saved=self.seed()
        selected,info=tr.tunnel_exit_rejoin(self.dense(ego),ego,tunnel,[],[],
            lambda p,h:dict(wall=h,lane=h,obstacle=h),previous_saved=saved,now=3.,budget_s=.5)
        self.assertIsNotNone(selected)
        self.assertFalse(info['reused'])
        self.assertTrue(info['committed_invalidated'])

    def test_portal_complete_uses_final_measured_heading_instead_of_legacy_or(self):
        runtime=ExitIntegrationTests().runtime()
        runtime.test_route[:,1]=0.
        path=gui.RosPathPublisher.resample(route(0.,60.))
        plan=dict(path=path,kind='rejoin',generator='portable',direction=1.,offset_m=0.,
                  exit_rejoin=True,curvature=0.,speed_cap_mps=10.,safe_horizon_m=60.,hold_until_m=0.)
        measured=dict(offset_m=.0839,heading_error_deg=3.62,complete=False)
        with patch.object(tr,'tunnel_exit_rejoin',return_value=(plan,dict(preparation_cap_kmh=None))),\
             patch.object(tr,'tunnel_route_alignment',return_value=measured):
            runtime.infer_tunnel(runtime.test_status,runtime.test_status['ego'])
        self.assertFalse(runtime._web_record['tunnel']['rejoin_complete'])
        self.assertFalse(runtime._web_record['tunnel']['exit_rejoin']['measured']['complete'])


if __name__ == '__main__':
    unittest.main()
