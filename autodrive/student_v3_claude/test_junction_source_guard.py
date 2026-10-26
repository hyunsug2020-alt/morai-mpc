"""Real recorded source revision and atomic publication boundary regressions."""
import copy
import json
from pathlib import Path
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from driving_plan import DrivingPlan
from ego_body import EGO_FRONT_M
from junction_source_guard import frame_identity, current_frame
from live_path_safety import LivePathSafety
from stop_line import competition_stop_controller

LINE = dict(id='entry', node_id='A1256W000709', station_m=50., xy=[50., 0.])


class AtomicReceiver:
    def __init__(self, record):
        self.lock = threading.Lock()
        self.record = copy.deepcopy(record)
        self.received = 99.95

    def replace(self, record):
        with self.lock:
            self.record = copy.deepcopy(record)
            self.received = 100.

    def snapshot(self):
        with self.lock:
            record = copy.deepcopy(self.record)
            age = 100.-self.received
        ready = 0. <= age <= .35 and record.get('ready') is True and record.get('active') is True
        return [], ready, dict(record, age_s=age)


class Planner:
    def __init__(self):
        self.junction_committed = False
        self.junction = dict(yield_only=True, held=True, line_id='entry', line_station_m=50.)

    def latch_junction_commit(self):
        self.junction_committed = True


class JunctionSourceGuardTests(unittest.TestCase):
    def fixture(self, record=None, kind='launch'):
        record = record or dict(stamp=99.9, active=True, ready=True, vehicles=[])
        receiver = AtomicReceiver(record)
        controller = competition_stop_controller([LINE])
        ego = dict(x=49.-EGO_FRONT_M, y=0., heading_deg=0., speed_kmh=0.,
                   stamp=float(record['stamp'])+.1, age_s=.01, received=99.99)
        controller.update(ego['x'], 0., 100., False, observation_age_s=.01, empty_front_station=49.)
        provider = LivePathSafety.__new__(LivePathSafety)
        provider.runtime = SimpleNamespace(vehicle_receiver=receiver, stop_line_controller=controller,
            gap_planner=Planner(), lanes=SimpleNamespace(route_progress=lambda pose: pose['x']))
        provider.traffic = Planner()
        provider.junction_launch = SimpleNamespace(clear=lambda reason: None)
        request = dict(kind=kind, frame=frame_identity(record), expires_at=100.15,
                       line_id='entry', line_station_m=50., front_station_m=49., speed_mps=0., pose_age_s=.01)
        validation = dict(valid=True, pose_valid=True, steering_valid=True, blocked=False,
                          junction_source_launch=request, junction_live=dict(expires_at=100.15),
                          live_safety=dict(junction_traffic=dict(state='APPROACH_YIELD',gap_checked=True)))
        path = np.array([[ego['x'],0.],[50.,0.],[80.,0.]])
        plan = DrivingPlan(1,path,ego,ego,float(record['stamp']),.1,99.99,3.757768,60.,False,validation,'ROUTE BASE')
        return provider, plan, receiver, controller

    def test_checked_physical_ring_launch_at_265cm_is_consumed_at_the_publication_barrier(self):
        for advanced, gap, expected in ((True, 2.65, True), (False, 2.65, False),
                                        (True, 3.001, False)):
            provider, plan, receiver, gate = self.fixture()
            gate.active = dict(gate.active, yield_advance_m=7. if advanced else 0.,
                               yield_launch_capture_m=3.)
            plan.validation['junction_source_launch']['front_station_m'] = 50.-gap
            with self.subTest(advanced=advanced, gap=gap), \
                    patch('junction_source_guard.time.monotonic', return_value=100.):
                with provider.publication_source_guard(plan) as allowed:
                    self.assertEqual(allowed, expected)
                    if allowed:
                        self.assertTrue(plan.validation['junction_source_launch']['consumed'])
                        self.assertEqual(plan.validation['live_safety']['junction_traffic']['state'], 'ENTER')
            self.assertEqual(gate.released, expected)
            self.assertEqual(provider.runtime.gap_planner.junction_committed, expected)

    def test_recorded_new_complete_frame_blocks_old_go_before_consume(self):
        fixture = json.loads((Path(__file__).parent/'test_data'/'native61_junction_source_race.json').read_text())
        self.assertLess(fixture['new']['bag_receipt'],fixture['old_publication']['bag_receipt'])
        self.assertEqual(fixture['old_publication']['junction']['ring']['action'],'GO')
        self.assertEqual(fixture['new_publication']['junction']['ring']['action'],'WAIT')
        provider,plan,receiver,gate=self.fixture(fixture['old']['status'])
        receiver.replace(fixture['new']['status'])
        with patch('junction_source_guard.time.monotonic',return_value=100.):
            self.assertEqual(current_frame(receiver,plan.validation['junction_source_launch']['frame']),(False,'changed'))
            with provider.publication_source_guard(plan) as allowed:
                self.assertFalse(allowed)
        self.assertFalse(gate.released or provider.traffic.junction_committed or provider.runtime.gap_planner.junction_committed)

    def test_same_stamp_body_revision_is_not_the_same_source(self):
        provider,plan,receiver,gate=self.fixture()
        changed=copy.deepcopy(receiver.record);changed['vehicles']=[dict(uid=9,velocity=[8.,0.])]
        receiver.replace(changed)
        with patch('junction_source_guard.time.monotonic',return_value=100.):
            with provider.publication_source_guard(plan) as allowed:self.assertFalse(allowed)
        self.assertFalse(gate.released)

    def test_readiness_loss_at_same_stamp_is_never_authorized(self):
        for key in ('ready','active'):
            for value in ('missing',None,False):
                with self.subTest(key=key,value=value):
                    provider,plan,receiver,gate=self.fixture();changed=copy.deepcopy(receiver.record)
                    if value=='missing':changed.pop(key)
                    else:changed[key]=value
                    receiver.replace(changed)
                    with patch('junction_source_guard.time.monotonic',return_value=100.):
                        with provider.publication_source_guard(plan) as allowed:self.assertFalse(allowed)
                    self.assertFalse(gate.released)

    def test_receiver_without_atomic_lock_fails_closed(self):
        provider,plan,receiver,gate=self.fixture();provider.runtime.vehicle_receiver=SimpleNamespace(record=receiver.record,received=99.95)
        with provider.publication_source_guard(plan) as allowed:self.assertFalse(allowed)
        self.assertFalse(gate.released)

    def test_latest_callback_cannot_interleave_comparison_and_two_writes(self):
        provider,plan,receiver,gate=self.fixture();attempt=threading.Event();done=threading.Event();writes=[]
        changed=dict(receiver.record,stamp=100.)
        def callback():
            attempt.set();receiver.replace(changed);done.set()
        with patch('junction_source_guard.time.monotonic',return_value=100.):
            with provider.publication_source_guard(plan) as allowed:
                self.assertTrue(allowed);thread=threading.Thread(target=callback);thread.start()
                self.assertTrue(attempt.wait(.2));self.assertFalse(done.wait(.01))
                writes.extend(['waypoints','velocity']);provider.note_published(plan,plan.path,plan.allowed_mps,100.)
                self.assertTrue(gate.released and provider.runtime.gap_planner.junction_committed)
                self.assertEqual(plan.validation['live_safety']['junction_traffic']['state'],'ENTER')
            thread.join(.2)
        self.assertTrue(done.is_set());self.assertEqual(writes,['waypoints','velocity'])

    def test_consumed_but_expired_zero_delivery_cancels_pending_commit(self):
        provider,plan,receiver,gate=self.fixture();clock=[100.]
        with patch('junction_source_guard.time.monotonic',side_effect=lambda:clock[0]):
            with provider.publication_source_guard(plan) as allowed:
                self.assertTrue(allowed);self.assertTrue(gate.released)
                clock[0]=100.2
                self.assertFalse(plan.junction_motion_fresh())
                zero=plan.decided(allowed_mps=0.,emergency=True)
                provider.note_published(zero,zero.path,0.,100.2)
                self.assertFalse(gate.released or provider.traffic.junction_committed or provider.runtime.gap_planner.junction_committed)

    def test_publication_exception_revokes_preline_pending_launch(self):
        provider,plan,receiver,gate=self.fixture()
        with patch('junction_source_guard.time.monotonic',return_value=100.):
            with self.assertRaisesRegex(RuntimeError,'ROS write'):
                with provider.publication_source_guard(plan) as allowed:
                    self.assertTrue(allowed);raise RuntimeError('ROS write')
        self.assertFalse(gate.released or provider.traffic.junction_committed or provider.runtime.gap_planner.junction_committed)

    def test_physical_crossing_not_delivered_still_revokes_only_this_transaction(self):
        provider,plan,receiver,gate=self.fixture()
        plan.validation['junction_source_launch']['front_station_m']=50.01
        plan.control_ego['x']=50.01-EGO_FRONT_M
        with patch('junction_source_guard.time.monotonic',return_value=100.):
            with provider.publication_source_guard(plan) as allowed:
                self.assertTrue(allowed);self.assertTrue(gate.released)
                self.assertFalse(gate.yield_launch_pending)
                zero=plan.decided(allowed_mps=0.,emergency=True)
                provider.note_published(zero,zero.path,0.,100.)
        self.assertFalse(gate.released or provider.traffic.junction_committed or provider.runtime.gap_planner.junction_committed)

    def test_late_rejection_of_delivered_request_cannot_undo_previous_entry(self):
        provider,plan,receiver,gate=self.fixture()
        with patch('junction_source_guard.time.monotonic',return_value=100.):
            with provider.publication_source_guard(plan) as allowed:
                self.assertTrue(allowed);provider.note_published(plan,plan.path,plan.allowed_mps,100.)
            provider.reject_source_publication(plan, 'old request rejected later')
        self.assertTrue(gate.released and provider.traffic.junction_committed and provider.runtime.gap_planner.junction_committed)

    def test_stale_or_future_receipt_never_consumes(self):
        for received in (99.,100.001,float('nan')):
            with self.subTest(received=received):
                provider,plan,receiver,gate=self.fixture();receiver.received=received
                with patch('junction_source_guard.time.monotonic',return_value=100.):
                    with provider.publication_source_guard(plan) as allowed:self.assertFalse(allowed)
                self.assertFalse(gate.released)

    def test_provisional_go_keeps_commit_deferred(self):
        provider,plan,receiver,gate=self.fixture(kind='provisional')
        with patch('junction_source_guard.time.monotonic',return_value=100.):
            with provider.publication_source_guard(plan) as allowed:self.assertTrue(allowed)
        self.assertFalse(gate.released or provider.traffic.junction_committed)

    def test_profile_cannot_commit_when_static_stop_precedes_required_rear_clearance(self):
        from test_junction_entry_profile import EntryProfileGuiTests
        from virtual_sim import SimulatedVehicleReceiver
        for raw_forward, blocked in ((15., True), (30., False)):
            fixture,runtime,captured=EntryProfileGuiTests().wait_runtime()
            with patch('gui.crossing_caution',return_value=None):
                fixture.step(runtime,0)
            publisher=runtime.path_publisher;ego=runtime.test_status['ego']
            receiver=SimulatedVehicleReceiver();runtime.vehicle_receiver=receiver
            runtime.object_receiver.objects=[]
            runtime.object_receiver.observation_stamp=100.
            runtime.lidar.pts=np.array([[raw_forward,0.]])
            provider=LivePathSafety(runtime);publisher._safety_provider=provider
            with self.subTest(raw_forward=raw_forward), patch('time.monotonic',return_value=100.):
                receiver.publish_rows([fixture.car(ego['x']+10.,35.,vy=-3.,heading=-90.)],stamp=100.)
                publisher.recheck_active_path()
                plan=publisher._active_decision_plan
                live=plan.validation['live_safety']
                self.assertEqual(live['junction_traffic']['ring']['action'],'GO')
                proof=live['junction_rear_travel']
                self.assertEqual(proof['accepted'],not blocked)
                if blocked:
                    self.assertLess(proof['clear_m'],proof['required_m'])
                    self.assertEqual(plan.allowed_mps,0.)
                    self.assertTrue(plan.emergency)
                    self.assertFalse(runtime.gap_planner.junction_committed)
                else:
                    self.assertGreaterEqual(proof['clear_m'],proof['required_m'])
                    self.assertGreater(plan.allowed_mps,0.)
                    self.assertTrue(runtime.gap_planner.junction_committed)

    def test_first_actual_live_wait_keeps_baseline_profile_for_a_new_timer_go(self):
        from test_junction_entry_profile import EntryProfileGuiTests, entry_profile_ceiling
        from virtual_sim import SimulatedVehicleReceiver
        fixture, runtime, captured = EntryProfileGuiTests().wait_runtime()
        ego = runtime.test_status['ego']
        receiver = SimulatedVehicleReceiver()
        runtime.vehicle_receiver = receiver
        runtime.object_receiver.observation_stamp = 100.
        publisher = runtime.path_publisher
        provider = LivePathSafety(runtime)
        publisher._safety_provider = provider
        clock = [100.]
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch('gui.crossing_caution', return_value=None):
            receiver.publish_rows([fixture.car(ego['x']+10., 6., vy=-8., heading=-90.)],
                stamp=100.)
            first = fixture.step(runtime, 0)
            active = publisher._active_decision_plan
            self.assertEqual(first['traffic_plan']['ring']['action'], 'WAIT')
            self.assertEqual(active.validation['live_safety']['junction_traffic']['ring']
                ['action'], 'WAIT')
            self.assertEqual(active.allowed_mps, 0.)
            self.assertTrue(active.emergency)
            self.assertTrue(active.validation['junction_entry_profile_refreshed'])
            self.assertFalse(runtime.gap_planner.junction_committed)
            created = publisher._safety_baseline['created']
            world_path = publisher._safety_baseline['world_path'].copy()
            baseline = publisher._safety_baseline['decision']
            self.assertEqual(baseline.allowed_mps, 0.)
            self.assertGreater(entry_profile_ceiling(baseline, 100.), 0.)
            # The same car advances 2 m in .25 s at its measured 8 m/s.
            # The timer rechecks a complete source frame, rather than inheriting
            # the first publication's WAIT or treating the profile as permission.
            clock[0] = 100.25
            ego['stamp'] = 100.25
            runtime.object_receiver.observation_stamp = 100.25
            receiver.publish_rows([fixture.car(ego['x']+10., 4., vy=-8., heading=-90.)],
                stamp=100.25)
            publisher.recheck_active_path()
            active = publisher._active_decision_plan
            self.assertGreater(active.allowed_mps, 0.)
            self.assertFalse(active.emergency)
            self.assertTrue(active.validation['junction_entry_profile_accepted'])
            traffic = active.validation['live_safety']['junction_traffic']
            self.assertEqual((traffic['state'], traffic['ring']['action']), ('ENTER', 'GO'))
            self.assertTrue(runtime.gap_planner.junction_committed)
            self.assertTrue(active.validation['junction_source_launch']['published'])
            promoted = publisher._safety_baseline['decision']
            self.assertIsNot(promoted, baseline)
            self.assertGreater(promoted.allowed_mps, 0.)
            self.assertFalse(promoted.emergency)
            self.assertGreater(entry_profile_ceiling(promoted, 100.25), 0.)
            for tick in range(1, 6):
                clock[0] = 100.25+.05*tick
                ego['stamp'] = clock[0]
                runtime.object_receiver.observation_stamp = clock[0]
                runtime.lidar.scan_snapshot = lambda: dict(stamp=clock[0], age_s=.01,
                    obstacle_points=runtime.lidar.pts.copy())
                receiver.publish_rows([fixture.car(ego['x']+10., 4.-.4*tick,
                    vy=-8., heading=-90.)], stamp=clock[0])
                publisher.recheck_active_path()
                continued = publisher._active_decision_plan
                self.assertGreater(continued.allowed_mps, 0., (tick, continued.stop_reason))
                self.assertFalse(continued.emergency, (tick, continued.stop_reason))
                self.assertTrue(runtime.gap_planner.junction_committed)
                self.assertTrue(runtime.stop_line_controller.released)
                self.assertNotIn('yield_launch_interrupted', continued.validation)
                self.assertEqual(publisher._safety_baseline['created'], created)
                np.testing.assert_array_equal(publisher._safety_baseline['world_path'], world_path)

    def test_current_stationary_semantic_cap_bounds_prospective_wait_profile(self):
        from test_junction_entry_profile import EntryProfileGuiTests
        from virtual_sim import SimulatedVehicleReceiver
        for current_cap in (0., .8):
            fixture,runtime,captured=EntryProfileGuiTests().wait_runtime()
            with patch('gui.crossing_caution',return_value=None):
                fixture.step(runtime,0)
            publisher=runtime.path_publisher
            original=captured[-1].decided(allowed_mps=current_cap,emergency=current_cap<=0.)
            publisher._safety_baseline['decision']=original
            receiver=SimulatedVehicleReceiver();runtime.vehicle_receiver=receiver
            ego=runtime.test_status['ego']
            obstacle=(ego['x']+5.,ego['y']+4.,1.,'static_person','obstacle',0.,
                      1.,1.,0.,0.,0.,91,0.,None,False,0.,None,
                      dict(pedestrian_static_reason='current stationary semantic evidence'))
            runtime.object_receiver.objects=[obstacle]
            runtime.object_receiver.observation_stamp=100.
            runtime.standing_obstacle_objects=lambda objects,now: list(objects)
            provider=LivePathSafety(runtime);provider.owner='ROAD';publisher._safety_provider=provider
            requested=[];real_check=provider.guard.check
            def observe(*args,**kwargs):
                requested.append(kwargs['requested_mps'])
                return real_check(*args,**kwargs)
            provider.guard.check=observe
            with self.subTest(current_cap=current_cap), patch('time.monotonic',return_value=100.):
                receiver.publish_rows([],stamp=100.)
                publisher.recheck_active_path()
                self.assertTrue(requested)
                self.assertLessEqual(max(requested),current_cap)
                self.assertLessEqual(publisher._active_decision_plan.allowed_mps,current_cap)
                if current_cap <= 0.:
                    self.assertFalse(runtime.gap_planner.junction_committed)

    def test_committed_following_and_nonplan_calls_keep_existing_behavior(self):
        provider,plan,receiver,gate=self.fixture();plan=plan.decided(junction_source_launch=None)
        provider.runtime.vehicle_receiver=SimpleNamespace()
        with provider.publication_source_guard(plan) as allowed:self.assertTrue(allowed)
        with provider.publication_source_guard(None) as allowed:self.assertTrue(allowed)


if __name__=='__main__':unittest.main()
