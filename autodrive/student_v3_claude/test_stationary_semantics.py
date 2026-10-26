"""Evidence, simulator-time stages and immediate restart protection on roads."""
import unittest
import threading
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

import gui
import model_avoidance as ma
import ped_guard as pg
from perception.stationary_semantics import (StaticPedestrianEvidence, annotate_static_candidates,
    metadata, with_metadata, STATIC_EVIDENCE_SIM_S)
from path_safety import PathSafety
from test_ped_guard import EGO, person
import test_pedestrian_deadlock as deadlock
from test_pedestrian_deadlock import Standing, PATH, corridor, host


def fusion_person(f=10., l=0., height=.84, length=.97, width=.3):
    o = list(person(f, l, 0., 0., uid=75))
    o[2], o[3], o[6], o[7] = max(length, width), 'fusion_pedestrian', length, width
    return with_metadata(o, height_m=height, raw_size_m=[length, width])


def low_obstacle(f=9.56, l=.06, height=.93):
    return with_metadata((f, l, 1.28, 'fusion_obstacle', 'obstacle', 0., 1.28, 1.1, 0., 0., 0., 38),
                         height_m=height, raw_size_m=[1.28, 1.1])


def evidence(objects, rtf=1., duration=2.2):
    e = StaticPedestrianEvidence()
    objects = annotate_static_candidates(objects)
    for t in np.arange(0., duration/rtf+.001, .1):
        e.update(objects, float(t), float(t), rtf)
    return e, objects, float(t)


class ShapeEvidenceTests(unittest.TestCase):
    def test_ros_receiver_preserves_lidar_height_and_unpadded_shape(self):
        receiver = gui.RosObjectReceiver.__new__(gui.RosObjectReceiver)
        receiver.lock = threading.Lock()
        receiver.objects, receiver.received = [], 0.
        receiver._tracks, receiver._motion, receiver._turn, receiver._position_history = {}, {}, {}, {}
        p = SimpleNamespace(position=SimpleNamespace(x=10., y=0.),
            size=SimpleNamespace(x=.97, y=.3, z=.84), velocity=SimpleNamespace(x=0., y=0.),
            heading=0., unique_id=75, name='fusion_pedestrian')
        with patch('gui.time.monotonic', return_value=100.):
            receiver.on_objects(SimpleNamespace(npc_list=[], pedestrian_list=[p], obstacle_list=[]))
        self.assertEqual(metadata(receiver.objects[0]), dict(height_m=.84, raw_size_m=[.97, .3]))
        self.assertEqual(receiver.objects[0][7], .6)  # the existing protective minimum stays

    def test_low_c_cluster_requires_two_simulator_seconds_of_fresh_positions(self):
        for rtf in (.6, .85, 1.):
            e = StaticPedestrianEvidence()
            obj = annotate_static_candidates([fusion_person()])[0]
            first = None
            for t in np.arange(0., 4., .1):
                e.update([obj], float(t), float(t), rtf)
                if e.reason(obj, float(t)) and first is None:
                    first = t
            self.assertGreaterEqual(first*rtf, STATIC_EVIDENCE_SIM_S-1e-6)
            self.assertLess(first*rtf, STATIC_EVIDENCE_SIM_S+.11)

    def test_missing_height_normal_human_shape_and_unknown_source_keep_human(self):
        for obj in (person(10., 0., 0., 0.), fusion_person(height=1.7),
                    fusion_person(length=.6, width=.6),
                    with_metadata(person(10., 0., 0., 0.), height_m=.84, raw_size_m=[1., .3])):
            e, rows, t = evidence([obj], duration=45.)
            self.assertIsNone(e.reason(rows[0], t))

    def test_overlapping_low_box_is_captured_before_dedupe_and_union_is_retained(self):
        # C: person centre and its obstacle segment differ by 0.44 m.
        p, box = fusion_person(length=.6), low_obstacle()
        runtime = host()
        for t in np.arange(0., 2.31, .1):
            rows = runtime.deduplicate_objects([p, box], float(t))
        obj = next(o for o in rows if o[11] == 75)
        self.assertEqual(obj[4], 'obstacle')
        self.assertEqual(metadata(obj)['pedestrian_static_reason'], 'overlapping_low_static_box')
        self.assertGreaterEqual(obj[6], box[6])
        self.assertGreaterEqual(obj[7], box[7])
        self.assertIn(box, rows)

    def test_tall_person_fragment_and_adjacent_low_box_are_not_static_evidence(self):
        p = fusion_person(height=1.7, length=.6, width=.6)
        for box in (low_obstacle(height=1.5), low_obstacle(l=1.5)):
            e, rows, t = evidence([p, box])
            self.assertIsNone(e.reason(rows[0], t))

    def test_explicit_tall_human_cannot_be_overridden_by_an_overlapping_low_box(self):
        e, rows, t = evidence([fusion_person(height=1.7, length=.6, width=.6), low_obstacle()])
        self.assertIsNone(e.reason(rows[0], t))

    def test_support_box_does_not_hide_a_person_restart_or_new_tall_shape(self):
        for failure in ('translation', 'height'):
            e, rows, t = evidence([fusion_person(length=.6), low_obstacle()])
            self.assertIsNotNone(e.reason(rows[0], t))
            changed = list(rows[0])
            if failure == 'translation':
                changed[1] += .16
            else:
                changed[17] = dict(metadata(changed), height_m=1.7)
            self.assertIsNone(e.reason(annotate_static_candidates([changed, rows[1]])[0], t+.1))
            e.update(annotate_static_candidates([changed, rows[1]]), t+.1, t+.1, 1.)
            self.assertIsNone(e.reason(changed, t+.1))

    def test_slow_translation_cannot_accumulate_shape_evidence_beside_a_fixed_box(self):
        e = StaticPedestrianEvidence()
        for t in np.arange(0., 2.31, .1):
            p = fusion_person(length=.6, l=.16*float(t))
            rows = annotate_static_candidates([p, low_obstacle()])
            e.update(rows, float(t), float(t), 1.)
            self.assertIsNone(e.reason(rows[0], float(t)))

    def test_speed_translation_missing_frame_clock_rewind_and_gap_revoke(self):
        for failure in ('speed', 'translation', 'missing', 'rewind', 'gap'):
            e, rows, t = evidence([fusion_person()])
            obj = rows[0]
            self.assertIsNotNone(e.reason(obj, t))
            changed = list(obj)
            if failure == 'speed':
                changed[5], changed[10] = .4, .4
            if failure == 'translation':
                changed[1] += .16
            if failure == 'missing':
                e.update([], t+.1, t+.1, 1.)
            when = t-1. if failure == 'rewind' else t+1. if failure == 'gap' else t+.1
            if failure in ('speed', 'translation'):
                self.assertIsNone(e.reason(changed, when))  # publication-time check before update
            e.update([changed], when, when, 1.)
            self.assertIsNone(e.reason(changed, when))

    def test_repeated_sensor_stamp_cannot_earn_or_refresh_evidence(self):
        e = StaticPedestrianEvidence()
        obj = annotate_static_candidates([fusion_person()])[0]
        for t in np.arange(0., 10., .1):
            e.update([obj], 100., float(t), 1.)
            self.assertIsNone(e.reason(obj, float(t)))

    def test_new_id_cannot_inherit_shape_confirmation(self):
        e, rows, t = evidence([fusion_person()])
        changed = list(rows[0]); changed[11] = 76
        self.assertIsNone(e.reason(changed, t))
        e.update([changed], t+.1, t+.1, 1.)
        self.assertEqual(e.tracks[76]['elapsed'], 0.)


class StagedWaitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Production configures the portable package with the host's vectorized
        # SAT before starting its bounded road search.
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)

    def wait(self, duration, rtf=1.):
        wait = pg.StandingBypassWait()
        p = person(10., 0., 0., 0.)
        for t in np.arange(0., duration/rtf+.001, .1):
            wait.update([p], EGO, float(t), rtf, Standing(), {16})
        return wait, p, float(t)

    def test_stage_boundaries_are_per_person_and_simulator_seconds(self):
        for rtf in (.6, 1.):
            for elapsed, gap, static in ((6.8, None, False), (7.2, .8, False),
                                         (19.8, .8, False), (20.2, .6, False),
                                         (39.8, .6, False), (40.2, .6, True)):
                wait, p, t = self.wait(elapsed, rtf)
                policy = wait.policy(p, t)
                if gap is None:
                    self.assertIsNone(policy)
                else:
                    self.assertEqual((policy['gap_m'], policy['static']), (gap, static))
                    self.assertEqual(policy['cap_mps'], 5./3.6)
                    self.assertIsNone(wait.policy(person(10., 0., 0., 0., uid=17), t))

    def test_timeout_does_not_survive_motion_or_a_new_location(self):
        wait, p, t = self.wait(40.2)
        for changed in (person(10., 0., 0., .3), person(10., -.16, 0., 0.)):
            self.assertIsNone(wait.policy(changed, t+.1))

    def test_small_fresh_translation_revokes_timeout_after_wait_update_too(self):
        wait, p, t = self.wait(40.2)
        moved = person(10., -.16, 0., 0.)
        wait.update([moved], EGO, t+.1, 1., Standing(), {16})
        self.assertIsNone(wait.policy(moved, t+.1))
        wait.update([moved], EGO, t+.2, 1., Standing(), {16})
        self.assertIsNone(wait.policy(moved, t+.2))

    def test_repeated_and_rewound_source_stamp_cannot_earn_wait(self):
        wait = pg.StandingBypassWait(); p = person(10., 0., 0., 0.)
        for t in np.arange(0., 45., .1):
            wait.update([p], EGO, float(t), 1., Standing(), {16}, stamp=100.)
        self.assertEqual(wait.tracks[16]['elapsed'], 0.)
        wait, p, t = self.wait(40.2)
        wait.update([p], EGO, t+.1, 1., Standing(), {16}, stamp=100.)
        wait.update([p], EGO, t+.2, 1., Standing(), {16}, stamp=99.)
        self.assertIsNone(wait.policy(p, t+.2))

    def test_each_stage_checks_a_complete_body_path_and_keeps_blocked_stop(self):
        for gap in (.8, .6):
            p = person(10., 0., 0., 0.)
            plan, _ = ma.plan_standing_bypass(PATH, EGO, [p], corridor(), lambda p,h:h, [p],
                person_clearances={16:gap}, speed_cap_mps=5./3.6, budget_s=2.)
            self.assertFalse(plan.get('blocked'))
            self.assertGreaterEqual(ma.standing_person_gap(plan['path'], EGO, [p]), gap)
            self.assertLessEqual(plan['speed_cap_mps'], 5./3.6)
            blocked, _ = ma.plan_standing_bypass(PATH, EGO, [p], corridor(), lambda p,h:0., [p],
                person_clearances={16:gap}, speed_cap_mps=5./3.6, budget_s=2.)
            self.assertTrue(blocked.get('blocked'))

    def test_timeout_classification_does_not_reset_its_wait_next_frame(self):
        runtime = host(); wait, p, t = self.wait(40.2)
        runtime._standing_bypass_wait = wait
        runtime._ped_waiting = {16:t}
        with patch.object(gui, 'model_avoidance', ma, create=True):
            for when in (t+.1, t+.2):
                rows = runtime.deduplicate_objects([p], when)
                self.assertEqual(rows[0][4], 'obstacle')
                lanes = corridor()
                plan, _, _ = runtime.model_static_plan(EGO, PATH, rows, np.empty((0,2)), lanes,
                    lambda *args:True, lanes.clear_distance, None, {}, when, True, True, 1.2)
                self.assertFalse(plan.get('blocked'))
                self.assertEqual(plan['static_person_ids'], [16])
                self.assertGreaterEqual(wait.tracks[16]['elapsed'], 40.)
        self.assertEqual(runtime.standing_obstacle_objects([person(10., -.3, 0., 0.)], t+.3)[0][4], 'pedestrian')

    def test_short_map_connector_does_not_create_a_false_gap_in_parallel_road(self):
        lanes = deadlock.StandingRoadTests().map()
        lanes.links.append(dict(idx='connector', road_id='C', link_type='0', width_start=3.5, width_end=3.5))
        lanes.geometry['connector'] = PATH[:2]
        lanes.link_of[:2] = 2
        lanes.n_links = 3
        lanes.link_trees['connector'] = lanes.link_trees['route']
        widened = ma.standing_road_corridor(lanes, EGO, PATH, corridor(1.75, -1.75))
        self.assertAlmostEqual(float(widened.left[0]), 5.25)
        self.assertAlmostEqual(float(widened.left[1]), 5.25)


class LiveRestorationTests(unittest.TestCase):
    def test_live_standing_bypass_checks_static_boxes_when_raw_scan_is_empty(self):
        from driving_plan import DrivingPlan
        import test_live_path_safety as live_tests
        fixture = live_tests.LivePathSafetyTests()
        runtime = fixture.attach(fixture.runtime())
        runtime.test_status['ego']['speed_kmh'] = 0.
        ego = dict(runtime.test_status['ego'])
        xy = gui.RosPathPublisher.global_xy(ego, 3.2, 0.)
        box = with_metadata((*xy, .4, 'fusion_static_candidate', 'obstacle', 0.,
            .4, .4, ego['heading_deg'], 0., 0., 10019),
            pedestrian_static_reason='standing_for_40_sim_s')
        runtime.object_receiver.objects = [box]
        provider = runtime.path_publisher._safety_provider
        validation = dict(valid=True, pose_valid=True, steering_valid=True, standing_bypass_ids=[10019])
        original = DrivingPlan(1, PATH, ego, ego, 99.9, .1, 100., 5./3.6, 60., False,
                               validation, 'STUDENT FORCED')
        world = [gui.RosPathPublisher.global_xy(ego, *p) for p in PATH]
        with patch('live_path_safety.time.monotonic', return_value=100.):
            result = provider(world, ego, 'STUDENT FORCED', original)
        self.assertEqual(result.validation['standing_static_clear_m'], 0.)
        self.assertTrue(result.emergency)
        self.assertEqual(result.allowed_mps, 0.)

    def test_selected_bypass_publication_rechecks_the_true_front_bumper(self):
        from driving_plan import DrivingPlan
        import test_live_path_safety as live_tests
        fixture = live_tests.LivePathSafetyTests()
        runtime = fixture.attach(fixture.runtime())
        ego = dict(runtime.test_status['ego'], speed_kmh=0.)
        runtime.test_status['ego'].update(ego)
        runtime.test_scan['obstacle_points'] = np.array([[3.2, 0.]])
        provider = runtime.path_publisher._safety_provider
        now = 100.
        path = PATH
        validation = dict(valid=True, pose_valid=True, steering_valid=True, standing_bypass_ids=[16])
        original = DrivingPlan(1, path, ego, ego, 99.9, .1, now, 5./3.6, 60., False,
                               validation, 'STUDENT FORCED')
        world = [gui.RosPathPublisher.global_xy(ego, *p) for p in path]
        with patch('live_path_safety.time.monotonic', return_value=now):
            result = provider(world, ego, 'STUDENT FORCED', original)
        self.assertAlmostEqual(provider.guard.front_m, pg.EGO_FRONT_M)
        self.assertAlmostEqual(provider.guard.rear_m, pg.EGO_REAR_M)
        self.assertTrue(result.emergency)
        self.assertEqual(result.allowed_mps, 0.)

    def test_fresh_restart_restores_human_motion_and_raw_collision_still_stops(self):
        runtime = host(); p = fusion_person(f=8., l=1.8)
        for t in np.arange(0., 2.31, .1):
            runtime.deduplicate_objects([p], float(t))
        converted = runtime.standing_obstacle_objects([p], float(t))
        self.assertEqual(converted[0][4], 'obstacle')
        walker = list(p); walker[1], walker[5], walker[10] = 1.1, 1., -1.
        fresh = runtime.standing_obstacle_objects([walker], float(t)+.1)
        self.assertEqual(fresh[0][4], 'pedestrian')
        guard = PathSafety()
        guard.check(PATH, EGO, [('semantic', converted, 100., .1)], 100.1, requested_mps=5./3.6,
                    dynamic_only=True, raw_clear_m=60.)
        stopped = guard.check(PATH, EGO, [('semantic', fresh, 100.1, .1)], 100.2,
                              requested_mps=5./3.6, dynamic_only=True, raw_clear_m=.1)
        self.assertTrue(stopped.emergency)
        self.assertEqual(stopped.cap_mps, 0.)

    def test_reclassification_replaces_lost_person_memory_without_clearing_other_stops(self):
        runtime = host(); p = fusion_person(f=6.3)
        for t in np.arange(0., 2.31, .1):
            runtime.deduplicate_objects([p], float(t))
        runtime._ped_front_memory = {75:dict(xy=p[:2], seen=t, clear_since=None)}
        decision = runtime.pedestrian_guard([p], EGO, PATH, float(t), points=np.array([[6.3, 0.]]))
        self.assertNotIn(75, runtime._ped_front_memory)
        self.assertIn(75, decision['ids'])
        obstacle_stop = dict(kind='stop', name='obstacle', reason='box on path')
        self.assertEqual(runtime.apply_pedestrian_guard(decision, obstacle_stop, EGO,
            np.array([[6.3, 0.]]), [p], float(t), PATH), obstacle_stop)


class StandingStaticSweepTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        ma.configure(gui.MAX_CONTROL_CURVATURE, gui.static_clear_distance)

    def test_small_c_box_between_coarse_poses_rejects_reusing_the_path(self):
        # C-prefix candidate from y=-109. The 0.25 m SAT reports 0.216 m
        # clearance; its front corner comes within 0.125 m between samples.
        path = np.array([[0., 0.], [.499989196, .003286965], [.999584676, .023388532],
            [1.497218959, .071869125], [1.990380764, .154024214],
            [2.476692321, .269804029], [2.954824567, .415560347],
            [3.424461533, .586664618], [3.887152939, .775847379],
            [4.343603010, .979813240], [4.795477421, 1.193823283],
            [5.244391817, 1.413995636]])
        direction = path[-1]-path[-2]
        direction /= np.linalg.norm(direction)
        path = np.vstack((path, path[-1]+np.arange(.5, 60., .5)[:, None]*direction))
        box = (6.987484954, -.134035137, 1.06, 'fusion_static_candidate', 'obstacle',
               0., 1.06, .4, -10.264, 0., 0., 10019)
        self.assertGreaterEqual(gui.static_clear_distance(path, EGO, [box], 60.,
            body_shift_m=ma.REAR_AXLE_SHIFT_M), 59.9)
        previous = dict(path=path, direction=1., offset_m=2.8, transition_m=10.,
                        standing_bypass_ids=[10019])
        plan, _ = ma.plan_standing_bypass(PATH, EGO, [box], corridor(100., -100.),
            lambda p,h:h, [], previous=previous, controlled_ids=[10019],
            static_person_ids=[10019], speed_cap_mps=5./3.6, allow_search=False)
        self.assertTrue(plan.get('blocked'))


if __name__ == '__main__':
    unittest.main()
