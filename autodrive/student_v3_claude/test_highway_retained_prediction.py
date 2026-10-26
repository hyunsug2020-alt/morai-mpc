"""A retained vehicle stays anchored to its actual observation and clock."""
import math
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from highway_three_lane import HighwayLane
from highway_planner import HighwayPlanner, local_points
from highway_sensor_alignment import align_highway_predictions, align_measured_cars
from highway_speed_plan import clearance_forecast_bound
from planning.highway.object_policy import (highway_measured_objects, highway_static_objects,
                                            highway_scan_mask_objects)
from planning.highway.motion_evidence import HighwayMotionEvidence
from planning.highway.prediction_safety import aligned_prediction_front_contact
from sim_time import to_sim_objects


def car(x, y=0., speed=5., age=0., uid=7):
    return (x, y, 7., 'lidar_car', 'npc', speed, 7., 2.2,
            0., speed, 0., uid, 0., None, True, age)


class RetainedOriginTests(unittest.TestCase):
    def setUp(self):
        self.evidence = HighwayMotionEvidence()
        self.ego = dict(x=0., y=0., heading_deg=0., speed_kmh=40.)
        points = np.column_stack((np.arange(-100., 400., .5), np.zeros(1000)))
        self.lanes = [HighwayLane('lane', 0, points, ('lane',), 300.)]

    def update(self, obj, stamp):
        return self.evidence.update([obj], stamp, self.lanes, self.ego)

    def test_missed_intermediate_observation_does_not_reanchor_latest_prediction(self):
        self.update(car(20.), 100.)
        # The detector saw the vehicle at 100.2, which the planner missed.
        # It predicted that observation to source time 100.4 (age 0.2).
        latest = car(22.8, speed=7., age=.2)
        objects, info = self.update(latest, 100.4)
        self.assertEqual(objects[0], latest)
        self.assertEqual(info['vehicles'][0]['evidence'], 'unseen observation prediction')
        self.assertEqual(self.evidence.tracks[7]['observed_stamp'], 100.)

    def test_epoch_timestamps_require_absolute_origin_equality(self):
        stamp = 1791735562.2082748
        self.update(car(20.), stamp)
        latest = car(23., speed=7., age=.2079010009765625)
        objects, _ = self.update(latest, 1791735562.6125917)
        self.assertEqual(objects[0], latest)

    def test_same_observation_retains_validated_motion_without_renewing_it(self):
        self.update(car(20.), 100.)
        self.evidence.tracks[7]['velocity'] = np.array([5., 0.])
        self.evidence.tracks[7]['lane_heading'] = 0.
        latest = car(22.8, speed=7., age=.4)
        objects, info = self.update(latest, 100.4)
        np.testing.assert_allclose(objects[0][:2], [22., 0.])
        self.assertEqual(objects[0][9:11], (5., 0.))
        self.assertEqual(info['vehicles'][0]['evidence'], 'retained motion')
        self.assertEqual(self.evidence.lane_headings[7], 0.)
        self.assertEqual(self.evidence.tracks[7]['observed_stamp'], 100.)

    def test_unseen_observation_does_not_inherit_old_lane_heading(self):
        self.update(car(20.), 100.)
        self.evidence.tracks[7]['lane_heading'] = 0.
        latest = list(car(22.8, speed=7., age=.2))
        latest[8], latest[10] = 12., 1.
        objects, _ = self.update(tuple(latest), 100.4)
        self.assertEqual(objects[0], tuple(latest))
        self.assertNotIn(7, self.evidence.lane_headings)

    def test_clock_rewind_discards_old_anchor(self):
        self.update(car(20.), 100.)
        latest = car(12., age=.2)
        objects, _ = self.update(latest, 99.)
        self.assertEqual(objects[0], latest)
        self.assertNotIn(7, self.evidence.tracks)


class PredictionAlignmentTests(unittest.TestCase):
    def test_position_total_age_and_uncertainty_advance_to_ego_clock(self):
        original = car(20., speed=5., age=.2)
        aligned, info = align_highway_predictions([original], 100., 100.2)
        self.assertAlmostEqual(aligned[0][0], 21.)
        self.assertAlmostEqual(aligned[0][15], .4)
        growth = 6.*(.4**2-.2**2)
        np.testing.assert_allclose(aligned[0][6:8], [7.+growth, 2.2+growth])
        self.assertEqual(info['aligned_count'], 1)
        self.assertEqual(info['aligned_indices'], [0])
        self.assertEqual(original, car(20., speed=5., age=.2))

    def test_rewind_never_shrinks_body_or_renews_prediction_ownership(self):
        original = car(20., age=.2)
        aligned, _ = align_highway_predictions([original], 100.1, 100.)
        self.assertAlmostEqual(aligned[0][0], 19.5)
        self.assertEqual(aligned[0][6:8], original[6:8])
        self.assertEqual(aligned[0][15], original[15])
        self.assertEqual(highway_measured_objects(aligned), [])

    def test_rewind_before_original_observation_is_rejected(self):
        original = car(20., age=.1)
        aligned, info = align_highway_predictions([original], 100.2, 100.)
        self.assertIs(aligned[0], original)
        self.assertEqual(info['rejected_count'], 1)

    def test_bounded_transport_delay_does_not_turn_a_moving_prediction_static(self):
        original = car(20., age=.9)
        aligned, info = align_highway_predictions([original], 100., 100.2)
        self.assertAlmostEqual(aligned[0][0], 21.)
        self.assertAlmostEqual(aligned[0][15], 1.1)
        growth = 6.*(1.1**2-.9**2)
        np.testing.assert_allclose(aligned[0][6:8], [7.+growth, 2.2+growth])
        self.assertEqual(highway_static_objects(aligned), [])
        self.assertEqual(highway_measured_objects(aligned), [])
        self.assertEqual(highway_scan_mask_objects(aligned), [])
        self.assertEqual(info['aligned_indices'], [0])
        self.assertEqual(info['expired_count'], 0)

    def test_transport_alignment_cannot_renew_an_expired_source_or_repeat_itself(self):
        expired = car(20., age=1.01)
        aligned, info = align_highway_predictions([expired], 100., 100.2)
        self.assertIs(aligned[0], expired)
        self.assertEqual(highway_static_objects(aligned), aligned)
        self.assertEqual(info['aligned_count'], 0)
        original = car(20., age=.9)
        first, _ = align_highway_predictions([original], 100., 100.2)
        same, info = align_highway_predictions(first, 100., 100.2)
        self.assertIs(same[0], first[0])
        self.assertEqual(info['aligned_count'], 0)
        repeated, info = align_highway_predictions(first, 100.2, 100.4)
        self.assertFalse(repeated[0][14])
        self.assertEqual(highway_static_objects(repeated), repeated)
        self.assertEqual(repeated[0][:14], first[0][:14])
        self.assertTrue(first[0][14])
        self.assertEqual(info['aligned_count'], 0)
        self.assertEqual(info['rejected_count'], 1)
        self.assertAlmostEqual(repeated[0][15], 1.1)
        # A malformed lineage also fails closed when its age is below 1 s.
        early, _ = align_highway_predictions([car(20., age=.2)], 100., 100.2)
        row = list(early[0])
        metadata = dict(row[18])
        lineage = dict(metadata['highway_prediction_alignment'])
        lineage['source_age_s'] = 1.01
        metadata['highway_prediction_alignment'] = lineage
        row[18] = metadata
        invalid = tuple(row)
        out, info = align_highway_predictions([invalid], 100., 100.2)
        self.assertFalse(out[0][14])
        self.assertEqual(highway_static_objects(out), out)
        self.assertEqual(info['rejected_count'], 1)

    def test_aligned_late_prediction_keeps_independent_static_geometry_and_front_contact(self):
        from lidar_corridor import without_moving_objects
        from gui import static_clear_distance
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=40.)
        route = np.column_stack((np.arange(0., 170.5, .5), np.zeros(341)))
        obstacle = (30., 0., 1., 'fusion_obstacle', 'obstacle', 0., 1., 1.,
                    0., 0., 0., 99, 0., None, None, 0.)
        aligned, info = align_highway_predictions([car(2., age=.9), obstacle], 100., 100.2)
        self.assertEqual(highway_static_objects(aligned), [obstacle])
        self.assertLess(static_clear_distance(route, ego, highway_static_objects(aligned), 170.), 30.)
        points = np.array([[3., 0.], [30., 0.]])
        np.testing.assert_array_equal(without_moving_objects(points,
            highway_measured_objects(aligned), ego), points)
        self.assertIsNotNone(aligned_prediction_front_contact(aligned, ego, info['aligned_indices']))

    def test_invalid_timestamps_cannot_keep_a_reused_alignment_dynamic(self):
        original = car(20., age=.9)
        first, _ = align_highway_predictions([original], 100., 100.2)
        for source, target in [(None, 100.2), (100., float('nan')),
                               (0., 100.2), (100., 100.351), (100.351, 100.)]:
            with self.subTest(source=source, target=target):
                out, info = align_highway_predictions(first, source, target)
                self.assertFalse(info['timing_valid'])
                self.assertFalse(out[0][14])
                self.assertEqual(highway_static_objects(out), out)
                self.assertEqual(info['rejected_count'], 1)
                self.assertEqual(out[0][:14], first[0][:14])
                self.assertEqual(out[0][15:], first[0][15:])
                self.assertTrue(first[0][14])
                raw, _ = align_highway_predictions([original], source, target)
                self.assertIs(raw[0], original)

    def test_invalid_alignment_lineage_cannot_promote_a_source_expired_vehicle(self):
        original = car(20., age=.9)
        aligned, _ = align_highway_predictions([original], 100., 100.2)
        late = aligned[0]
        self.assertEqual(highway_static_objects([late]), [])
        self.assertEqual(highway_static_objects([car(20., age=1.1)]), [car(20., age=1.1)])
        for field, value in [('source_age_s', 1.01), ('source_stamp', float('nan')),
                             ('target_stamp', 100.4), ('target_age_s', 1.3),
                             ('source_stamp', 100.3)]:
            with self.subTest(field=field):
                row = list(late)
                metadata = dict(row[18])
                lineage = dict(metadata['highway_prediction_alignment'])
                lineage[field] = value
                metadata['highway_prediction_alignment'] = lineage
                row[18] = metadata
                invalid = tuple(row)
                self.assertEqual(highway_static_objects([invalid]), [invalid])

    def test_alignment_metadata_is_immutable_and_survives_simulator_unit_conversion(self):
        raw_info = dict(height_m=1.51)
        extra = dict(detector='original')
        original = car(20., age=.9)+(None, raw_info, extra)
        aligned, _ = align_highway_predictions([original], 100., 100.2)
        self.assertIs(aligned[0][17], raw_info)
        self.assertEqual(extra, dict(detector='original'))
        self.assertIsNot(aligned[0][18], extra)
        converted = to_sim_objects(aligned, .5)
        self.assertEqual(converted[0][18], aligned[0][18])
        self.assertEqual(highway_static_objects(converted), [])
        self.assertEqual(highway_scan_mask_objects(converted), [])
        for invalid in (True, 123, 'unexpected', ['not', 'a', 'mapping']):
            row = car(20., age=.9)+(None, raw_info, invalid)
            out, info = align_highway_predictions([row], 100., 100.2)
            self.assertIs(out[0], row)
            self.assertEqual(info['rejected_count'], 1)

    def test_recorded_source_lifetime_crossings_preserve_moving_uncertainty(self):
        # Sensor collection follow-20261012T101217Z: these two measured movers
        # became fixed 25.5/28.25 m stop lines only after source-to-ego alignment.
        cases = [
            (1791800198.0673249, .12659382820129395, .9603366851806641,
             (75.76078722102162, -310.47559636876684, 10.4843421779442,
              'lidar_car', 'npc', 13.639962707170778, 10.4843421779442,
              7.614167293148057, 183.3422650838826, .07097113527330663,
              -13.639778068244643, 500028, 0., None, True, 1.086930513381958)),
            (1791800200.4934182, .07682394981384277, .9752376079559326,
             (74.30899264475951, -354.33378927311406, 10.67859913845075,
              'lidar_car', 'npc', 16.223648804941835, 10.67859913845075,
              7.793157358360418, 271.9069022563072, .2507720792688531,
              -16.221710572881943, 500030, 0., None, True, 1.0520615577697754)),
        ]
        for target, delay, source_age, expired in cases:
            with self.subTest(uid=expired[11]):
                self.assertEqual(highway_static_objects([expired]), [expired])
                row = list(expired)
                row[15] = source_age
                aligned, info = align_highway_predictions([tuple(row)], target-delay, target)
                self.assertEqual(highway_static_objects(aligned), [])
                np.testing.assert_allclose(aligned[0][:2],
                    np.asarray(expired[:2])+delay*np.asarray(expired[9:11]))
                growth = 6.*(expired[15]**2-source_age**2)
                np.testing.assert_allclose(aligned[0][6:8], np.asarray(expired[6:8])+growth)
                self.assertEqual(info['aligned_indices'], [0])
                self.assertEqual(highway_measured_objects(aligned), [])
                self.assertEqual(highway_scan_mask_objects(aligned), [])

    def test_fresh_vehicle_is_not_aligned_twice(self):
        original = car(20.)
        fresh, _ = align_measured_cars([original], 100., 100.2)
        aligned, info = align_highway_predictions(fresh, 100., 100.2)
        self.assertIs(aligned[0], fresh[0])
        self.assertAlmostEqual(aligned[0][0], 21.)
        self.assertEqual(info['aligned_count'], 0)

    def test_inferred_flow_keeps_its_separate_prediction_lifetime(self):
        for source in ('tracked', 'assumed'):
            with self.subTest(source=source):
                original = car(20., age=.9)+(source,)
                aligned, info = align_highway_predictions([original], 100., 100.2)
                self.assertIs(aligned[0], original)
                self.assertEqual(info['aligned_count'], 0)
                self.assertEqual(info['expired_count'], 0)
                self.assertEqual(highway_static_objects(aligned), [])
                self.assertEqual(highway_measured_objects(aligned), [])

    def test_invalid_unknown_and_non_lidar_objects_pass_through(self):
        original = car(20., age=.2)
        for index, value in [(14, False), (14, float('nan')), (15, -.1),
                             (15, float('nan')), (15, 1.1), (3, 'fusion_npc'),
                             (0, float('nan')), (6, 0.), (9, float('inf'))]:
            row = list(original)
            row[index] = value
            value = tuple(row)
            with self.subTest(index=index):
                aligned, info = align_highway_predictions([value], 100., 100.2)
                self.assertIs(aligned[0], value)
                self.assertEqual(info['aligned_count'], 0)
        for value in [(), original[:15], None]:
            aligned, _ = align_highway_predictions([value], 100., 100.2)
            self.assertIs(aligned[0], value)

    def test_invalid_or_unbounded_sensor_times_do_not_move_forecasts(self):
        original = car(20., age=.2)
        for source, target in [(0., 100.), (None, 100.), (100., float('nan')),
                               (100., 100.351), (100.351, 100.), (100., float('inf'))]:
            aligned, info = align_highway_predictions([original], source, target)
            self.assertIs(aligned[0], original)
            self.assertFalse(info['timing_valid'])

    def test_predicted_body_never_removes_near_static_returns(self):
        from lidar_corridor import without_moving_objects
        ego = dict(x=0., y=0., heading_deg=0.)
        points = np.array([[21., 0.], [18., 0.], [21., 2.]])
        aligned, _ = align_highway_predictions([car(20., age=.2)], 100., 100.2)
        np.testing.assert_array_equal(without_moving_objects(
            points, highway_measured_objects(aligned), ego), points)


class RecordedHighwayBrakingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = json.loads((Path(__file__).parent/'test_data'/
            'highway_retained_origin_20261012.json').read_text())

    def planner(self, frame, body_shift_m=None):
        path = local_points(np.asarray(frame['checked_path_world']), frame['ego'])
        route = np.column_stack((np.arange(0., 1000., .5), np.zeros(2000)))
        planner = HighwayPlanner(dict(from_idx=0, to_idx=1800, kmh=120.,
            lane_change_policy='free_three_lane', lane_change_goal=0,
            comfort_decel_mps2=2.5, comfort_jerk_mps3=4., emergency_jerk_mps3=12.), route,
            **({} if body_shift_m is None else dict(body_shift_m=body_shift_m)))
        planner.longitudinal.time_scale = frame['rtf']
        planner.longitudinal.sensor_age_s = frame['sensor_age_s']
        planner.longitudinal.fusion_sensor_age_s = frame['fusion_age_s']
        planner.longitudinal.static_sensor_age_s = frame['static_sensor_age_s']
        if frame['previous_ego'] is not None:
            planner.longitudinal.begin(frame['previous_ego'])
        planner.longitudinal.begin(frame['ego'])
        return planner, path

    def command(self, frame, objects, extra_static=None, acceleration=None, body_shift_m=None):
        planner, path = self.planner(frame, body_shift_m)
        if acceleration is not None:
            planner.longitudinal.acceleration = acceleration
        clear = min(clearance_forecast_bound(v, frame['clearance_horizon_m'])
                    for v in frame['clearances'].values())
        if extra_static is not None:
            clear = min(clear, extra_static)
        return planner.plan_speed(path, frame['ego'], to_sim_objects(objects, frame['rtf']),
            120./3.6, static_clear_m=clear, ceiling_mps=120./3.6)

    def fixed_first_objects(self):
        fixture, frame = self.fixture, self.fixture['frames'][0]
        objects = [tuple(o) for o in frame['objects']]
        at = next(i for i, obj in enumerate(objects) if obj[11] == fixture['uid'])
        latest = list(objects[at])
        latest[:2] = fixture['latest_detector']['position']
        latest[9:11] = fixture['latest_detector']['velocity']
        latest[5] = math.hypot(*latest[9:11])
        prior = fixture['prior_observation']
        evidence = HighwayMotionEvidence()
        evidence.tracks[fixture['uid']] = dict(observed_stamp=prior['stamp'],
            position=np.array(prior['position']), velocity=np.array(prior['velocity']))
        retained, _ = evidence.update([tuple(latest)], frame['vehicle_stamp'], [], frame['ego'])
        objects[at] = retained[0]
        aligned, _ = align_highway_predictions(objects, frame['vehicle_stamp'], frame['ego']['stamp'])
        return aligned

    def test_saved_first_emergency_reproduces_before_time_correction(self):
        frame = self.fixture['frames'][0]
        self.assertTrue(self.command(frame, [tuple(o) for o in frame['objects']])['emergency'])

    def test_legacy_hull_alone_would_release_first_emergency_after_clock_correction(self):
        # Counterfactual only: the runtime must also apply the physical guard.
        frame = self.fixture['frames'][0]
        fixed = self.fixed_first_objects()
        before = next(o for o in frame['objects'] if o[11] == self.fixture['uid'])
        after = next(o for o in fixed if o[11] == self.fixture['uid'])
        self.assertGreater(after[6], before[6])
        self.assertGreater(after[7], before[7])
        command = self.command(frame, fixed, body_shift_m=0.)
        self.assertFalse(command['emergency'])
        self.assertTrue(command['diagnostics']['reference_checked'])
        self.assertGreater(command['reference_mps'], 0.)
        # Counterfactual geometry still needs the newly checked scalar brake.
        self.assertLess(command['reference_mps']*3.6, frame['ego']['speed_kmh'])

    def test_legacy_hull_counterfactual_includes_measured_acceleration_lag(self):
        frame, fixed = self.fixture['frames'][0], self.fixed_first_objects()
        for acceleration in (-4., -2.5, -1., 0.):
            with self.subTest(acceleration=acceleration):
                command = self.command(frame, fixed, acceleration=acceleration, body_shift_m=0.)
                self.assertFalse(command['emergency'])
                self.assertTrue(command['diagnostics']['reference_checked'])
                self.assertGreater(command['reference_mps'], 0.)
                self.assertLess(command['reference_mps']*3.6, frame['ego']['speed_kmh'])

    def test_corrected_first_body_cannot_release_runtime_emergency(self):
        frame, fixed = self.fixture['frames'][0], self.fixed_first_objects()
        at = next(i for i, obj in enumerate(fixed) if obj[11] == self.fixture['uid'])
        contact = aligned_prediction_front_contact(fixed, frame['ego'], [at])
        self.assertEqual(contact['object_id'], self.fixture['uid'])
        self.assertLess(contact['separating_axis_gap_m'], 0.)
        # Even the original measured dimensions, without either uncertainty
        # shell, overlap the actual front; there is no evidence to shrink them.
        measured = list(fixed[at])
        measured[6:8] = self.fixture['latest_detector']['size'][:2]
        fixed[at] = tuple(measured)
        self.assertIsNotNone(aligned_prediction_front_contact(fixed, frame['ego'], [at]))
        self.assertTrue(self.command(frame, fixed)['emergency'])
        planner, path = self.planner(frame)
        command = planner.plan_speed(path, frame['ego'], to_sim_objects(fixed, frame['rtf']),
            120./3.6, emergency=True)
        self.assertTrue(command['emergency'])
        self.assertEqual(command['reference_mps'], 0.)

    def test_close_raw_static_hit_still_requires_stop_with_corrected_traffic(self):
        self.assertTrue(self.command(self.fixture['frames'][0],
            self.fixed_first_objects(), extra_static=3.75)['emergency'])

    def test_later_large_observed_body_and_recorded_raw_hit_remain_protected(self):
        for frame in self.fixture['frames'][1:]:
            with self.subTest(stamp=frame['wall_stamp']):
                self.assertTrue(self.command(frame, [tuple(o) for o in frame['objects']])['emergency'])


class FrontPredictionContactTests(unittest.TestCase):
    def setUp(self):
        self.ego = dict(x=0., y=0., heading_deg=0.)

    def test_actual_front_touch_is_blocked_but_positive_separation_passes(self):
        from ego_body import EGO_FRONT_M
        for offset, blocked in ((-1e-9, True), (0., True), (1e-9, False)):
            with self.subTest(offset=offset):
                body = car(EGO_FRONT_M+3.5+offset, age=.4)
                risk = aligned_prediction_front_contact([body], self.ego, [0])
                self.assertEqual(risk is not None, blocked)

    def test_rotated_body_uses_its_long_axis_and_world_ego_heading(self):
        body = list(car(8., age=.4))
        body[8] = 0.
        self.assertIsNone(aligned_prediction_front_contact([tuple(body)], self.ego, [0]))
        body[0], body[1], body[8] = 10., 27., 90.
        ego = dict(x=10., y=20., heading_deg=90.)
        self.assertIsNotNone(aligned_prediction_front_contact([tuple(body)], ego, [0]))
        body[8] = 0.
        self.assertIsNone(aligned_prediction_front_contact([tuple(body)], ego, [0]))

    def test_unaligned_and_rear_vehicles_do_not_gain_a_new_stop_rule(self):
        ahead, rear = car(6., age=.4), car(-1., age=.4)
        self.assertIsNone(aligned_prediction_front_contact([ahead, rear], self.ego, []))
        self.assertIsNone(aligned_prediction_front_contact([ahead, rear], self.ego, [1]))

    def test_separated_adjacent_lane_passes_with_the_whole_large_body(self):
        self.assertIsNone(aligned_prediction_front_contact([car(6., y=3.5, age=.4)], self.ego, [0]))


class RuntimePredictionClockTests(unittest.TestCase):
    def run_runtime(self, prediction, points):
        from gui import Runtime
        from highway_planner import NearFieldMemory
        from lidar_corridor import LidarCorridor
        from planning.highway.object_policy import highway_scan_mask_objects

        route = np.column_stack((np.arange(0., 1000., .5), np.zeros(2000)))
        runtime = Runtime.__new__(Runtime)
        runtime.highway = HighwayPlanner(dict(from_idx=0, to_idx=1800, kmh=90.), route)
        runtime.vehicle_receiver = SimpleNamespace(snapshot=lambda:
            ([prediction], True, dict(ready=True, stamp=100., age_s=.01, vehicles=[])))
        runtime.on_road_vehicles = lambda cars: cars
        runtime._highway_vehicle_motion = SimpleNamespace(
            update=lambda cars, *args: (cars, {}), camera_vehicles=lambda cars, *args: cars)
        runtime.object_receiver = SimpleNamespace(snapshot_with_time=lambda: ([], 100., .01))
        runtime.lidar = LidarCorridor()
        runtime.lidar.received = time.monotonic()
        runtime.lidar.pts = np.asarray(points, dtype=float).reshape(-1, 2)
        runtime.lidar.xy_snapshot = lambda: dict(points=runtime.lidar.pts,
            stamp=100., age_s=.01, received=runtime.lidar.received)
        ego = dict(x=0., y=0., heading_deg=0., speed_kmh=40., stamp=100.2, age_s=.01)
        runtime.ego_receiver = SimpleNamespace(at_stamp=lambda stamp: dict(ego, x=-1., stamp=stamp))
        runtime.near_field_memory = NearFieldMemory()
        runtime.lanes = SimpleNamespace(corridor=lambda *args:
            SimpleNamespace(clear_distance=lambda path, horizon: horizon),
            adjacent_offsets=lambda *args: {})
        runtime.receiver = SimpleNamespace(rates=lambda: {})
        runtime.actions = ['DRIVE']
        runtime.gallery, runtime.gallery_matrix = [], np.empty((0, 2))
        publisher = SimpleNamespace(_last_target_mps=0.,
            publish_camera_rates=lambda *args: None, publish_corridor=lambda *args: None)
        publisher.publish = lambda *args: setattr(publisher, '_last_target_mps', publisher.zone_mps)
        runtime.path_publisher = publisher
        seen_masks = []
        def masks(objects):
            seen_masks.append(list(objects))
            return highway_scan_mask_objects(objects)
        with tempfile.TemporaryDirectory() as directory, \
                patch.dict('os.environ', {'STUDENT_LOG_DIR': directory}), \
                patch('planning.highway.object_policy.highway_scan_mask_objects', masks):
            runtime.args = SimpleNamespace(status=Path(directory)/'status.json')
            dense = route[:640]
            self.assertTrue(runtime.infer_highway(
                dict(path=dict(nearest_idx=0, distance_m=0.)), ego,
                dense, dense[:20], dense[::10], dense[:20],
                np.empty(0), np.empty(0), np.empty(0), 0., time.perf_counter()))
            runtime._decision_log.close()
        return runtime._web_record, publisher, seen_masks

    def test_runtime_aligns_predictions_once_after_masks_and_keeps_raw_stop(self):
        record, publisher, seen_masks = self.run_runtime(car(30., age=.2), [[5., 0.], [30., 0.]])
        masked = next(o for o in seen_masks[0] if o[11] == 7)
        planned = next(o for o in record['object_tracks'] if o[11] == 7)
        self.assertEqual(masked[0], 30.)
        self.assertEqual(masked[15], .2)
        self.assertAlmostEqual(planned[0], 31.)
        self.assertAlmostEqual(planned[15], .4)
        info = record['highway']['sensor_alignment']['retained_traffic']
        self.assertEqual(info['aligned_count'], 1)
        self.assertTrue(record['emergency_stop'])
        self.assertEqual(publisher._last_target_mps, 0.)

    def test_runtime_actual_nose_contact_stops_with_no_raw_obstacle(self):
        record, publisher, _ = self.run_runtime(car(6.5, age=.2), [])
        risk = record['highway']['retained_prediction_front_contact']
        self.assertEqual(risk['object_id'], 7)
        self.assertLess(risk['separating_axis_gap_m'], 0.)
        self.assertEqual(record['reason'], 'highway retained vehicle overlaps actual front body')
        self.assertTrue(record['emergency_stop'])
        self.assertEqual(publisher._last_target_mps, 0.)


if __name__ == '__main__':
    unittest.main()
