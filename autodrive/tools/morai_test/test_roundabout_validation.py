import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

import roundabout_validation as validation


class RoundaboutValidationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        base = Path(self.temp.name)
        self.run, self.raw, self.source = base/'run', base/'raw', base/'source'
        for path in (self.run, self.raw, self.source):
            path.mkdir()
        self.gui = []
        self.ego = []
        x = 0.
        for index in range(73):
            t = 10.+.25*index
            mode = 'HIGHWAY' if 24. <= t < 26. else 'TUNNEL BYPASS' if t >= 26. else 'ROUTE BASE'
            self.gui.append(dict(t=t, ego=[x, 0.], mode=mode, stop_line=None,
                traffic_plan=dict(state='CRUISE', target_mps=2.),
                intersection_lidar=dict(active_region=dict(stop_station_m=10., exit_station_m=20.))))
            self.ego.append(dict(type='ego', t_us=round(t*1e6), x=x, y=0., heading_deg=0., vx=2., vy=0.))
            x += .5
        self.write(self.run/'run.json', dict(session_id='fixture', started_wall_ns=9e9,
            ended_wall_ns=29e9, exit_code=0))
        self.write(self.raw/'session.json', dict(session_id='fixture'))
        self.write(self.raw/'route.json', dict(frame_id='map',
            poses_xyz_qxyzw=[[float(x), 0., 0., 0., 0., 0., 1.] for x in range(61)]))
        self.write(self.raw/'collisions.json', dict(events=[], messages=0))
        self.write(self.run/'lap_complete.json', dict(lap_complete=True))
        hashes = {}
        for name in validation.CORE_SOURCES:
            path = self.source/name
            path.write_text(name+' original runtime\n')
            hashes[name] = validation.file_digest(path)
        self.write(self.run/'roundabout_source_snapshot.json',
            dict(captured_wall=9., scenario_sha256='a'*64, source_sha256=hashes))
        self.save_rows()

    def write(self, path, value):
        path.write_text(json.dumps(value))

    def save_rows(self):
        for path, rows in ((self.run/'student_gui_fixture.jsonl', self.gui),
                           (self.raw/'ego_raw.jsonl', self.ego)):
            path.write_text(''.join(json.dumps(row)+'\n' for row in rows))

    def evaluate(self, **options):
        return validation.evaluate_run(self.run, self.raw, stop_station_m=10.,
            source_root=self.source, **options)

    def codes(self, result):
        return {row['code'] for row in result['failures']}

    def test_actual_front_entry_and_entire_body_exit_are_required(self):
        result = self.evaluate(require_full_lap=True)
        self.assertEqual(result['verdict'], 'PASS')
        entry, exit_ = result['roundabout']['front_entry'], result['roundabout']['whole_body_exit']
        self.assertGreaterEqual(entry['front_station_m'], 10.)
        self.assertGreaterEqual(exit_['minimum_body_station_m'], 20.)
        self.assertGreater(exit_['t'], entry['t'])
        self.assertEqual(result['sectors']['highway']['min_actual_kmh'], 7.2)
        self.assertFalse(result['source_evidence']['validates_current_checkout'])

    def test_collision_free_but_never_entered_is_not_a_pass(self):
        for row in self.ego:
            row['x'] = 1.
        self.save_rows()
        result = self.evaluate()
        self.assertEqual(result['verdict'], 'FAIL')
        self.assertIn('roundabout_not_entered', self.codes(result))

    def test_native33_pattern_stalled_inside_fails_despite_no_collision(self):
        self.gui, self.ego = [], []
        for index in range(101):
            t = 10.+index*.5
            x = min(12., index)
            self.gui.append(dict(t=t, ego=[x, 0.], mode='ROUTE BASE', stop_line=None,
                traffic_plan=dict(state='CROSSING_CAUTION', target_mps=0.)))
            self.gui[-1]['yield'] = {'v_kmh': 0.}
            self.ego.append(dict(type='ego', t_us=int(t*1e6), x=x, y=0., heading_deg=0.,
                                 vx=0. if index >= 12 else 2., vy=0.))
        self.save_rows()
        result = self.evaluate()
        self.assertEqual(result['verdict'], 'FAIL')
        self.assertIn('long_roundabout_standstill', self.codes(result))
        self.assertIn('roundabout_body_did_not_clear_exit', self.codes(result))
        self.assertGreater(result['roundabout']['longest_standstill_wall_s'], 15.)

    def test_crossing_line_alone_does_not_prove_junction_exit(self):
        for row in self.ego:
            row['x'] = min(row['x'], 18.)
        self.save_rows()
        result = self.evaluate()
        self.assertIsNotNone(result['roundabout']['front_entry'])
        self.assertIsNone(result['roundabout']['whole_body_exit'])
        self.assertIn('roundabout_body_did_not_clear_exit', self.codes(result))

    def test_same_gui_frame_yield_cannot_raise_contact_cap(self):
        self.gui[30]['traffic_plan'] = dict(state='CROSSING_CAUTION', target_mps=1.)
        self.gui[30]['yield'] = dict(v_kmh=3.801)
        self.save_rows()
        result = self.evaluate()
        self.assertIn('following_only_contact_cap_override', self.codes(result))
        self.gui[30]['yield']['v_kmh'] = 3.8
        self.save_rows()
        self.assertEqual(self.evaluate()['verdict'], 'PASS')

    def test_legacy_surveyed_stop_line_is_reported_separately(self):
        self.gui[30].update(stop_line={'state': 'HOLD'},
            traffic_plan=dict(state='CROSSING_CAUTION', target_mps=0.))
        self.gui[30]['yield'] = dict(v_kmh=24.)
        self.save_rows()
        result = self.evaluate()
        self.assertEqual(result['verdict'], 'PASS')
        self.assertTrue(result['policy']['recorded_stop_line_present'])

    def test_cleanup_after_gui_end_is_excluded_from_all_speed_checks(self):
        last = self.ego[-1]
        for index in range(1, 121):
            self.ego.append(dict(last, t_us=int((28.+index*.25)*1e6), vx=0., vy=0.))
        self.save_rows()
        result = self.evaluate(require_full_lap=True)
        self.assertEqual(result['verdict'], 'PASS')
        self.assertTrue(result['driving_window']['cleanup_samples_excluded'])
        self.assertEqual(result['sectors']['tunnel']['stopped_samples'], 0)

    def test_paused_start_outside_junction_is_not_a_junction_stop(self):
        pause = [dict(type='ego', t_us=int(t*1e6), x=-100., y=0., heading_deg=0., vx=0., vy=0.)
                 for t in range(1, 10)]
        self.ego = pause+self.ego
        self.gui = [dict(t=float(t), mode='ROUTE BASE', ego=[-100., 0.]) for t in range(1, 10)]+self.gui
        self.save_rows()
        result = self.evaluate()
        self.assertEqual(result['verdict'], 'PASS')
        self.assertEqual(result['roundabout']['full_stop_wall_s'], 0.)
        self.assertEqual(result['roundabout']['front_entry']['speed_kmh'], 7.2)

    def test_full_lap_marker_requires_actual_highway_and_tunnel_observations(self):
        for row in self.gui:
            row['mode'] = 'ROUTE BASE'
        self.save_rows()
        result = self.evaluate(require_full_lap=True)
        self.assertEqual(result['verdict'], 'FAIL')
        self.assertEqual(sum(row['code'] == 'missing_actual_sector_coverage' for row in result['failures']), 2)

    def test_full_lap_fails_on_one_actual_sector_stop(self):
        self.ego[-2]['vx'] = 0.
        self.save_rows()
        result = self.evaluate(require_full_lap=True)
        self.assertIn('actual_sector_stop', self.codes(result))

    def test_source_hash_change_cannot_inherit_recorded_success(self):
        self.assertEqual(self.evaluate(compare_current_sources=True)['verdict'], 'PASS')
        (self.source/'junction_vehicle_motion.py').write_text('subsequent code change\n')
        result = self.evaluate(compare_current_sources=True)
        self.assertEqual(result['verdict'], 'FAIL')
        self.assertIn('current_source_version_mismatch', self.codes(result))
        self.assertEqual(self.evaluate()['verdict'], 'PASS')  # Explicitly recorded-version-only.
        self.assertFalse(self.evaluate()['source_evidence']['validates_current_checkout'])

    def test_missing_collision_or_route_evidence_and_running_job_are_incomplete(self):
        (self.raw/'collisions.json').unlink()
        result = self.evaluate()
        self.assertEqual(result['verdict'], 'INCOMPLETE')
        self.write(self.raw/'collisions.json', dict(events=[], messages=0))
        (self.raw/'route.json').unlink()
        self.assertEqual(self.evaluate()['verdict'], 'INCOMPLETE')

    def test_actual_collision_overrides_otherwise_complete_run(self):
        self.write(self.raw/'collisions.json', dict(events=[{'t': 18.}], messages=1))
        self.assertIn('collision', self.codes(self.evaluate()))

    def test_valid_json_with_invalid_route_schema_is_incomplete(self):
        self.write(self.raw/'route.json', [])
        self.assertEqual(self.evaluate()['verdict'], 'INCOMPLETE')

    def test_running_archive_is_not_completed_by_a_lap_marker(self):
        self.write(self.run/'run.json', dict(session_id='fixture', started_wall_ns=9e9))
        self.assertEqual(self.evaluate()['verdict'], 'INCOMPLETE')

    def test_missing_source_or_session_identity_cannot_pass(self):
        (self.run/'roundabout_source_snapshot.json').unlink()
        self.assertEqual(self.evaluate()['verdict'], 'INCOMPLETE')
        self.write(self.raw/'session.json', {})
        self.assertEqual(self.evaluate()['verdict'], 'INCOMPLETE')

    def test_missing_output_directory_keeps_json_stdout_and_nonzero_exit(self):
        with contextlib.redirect_stdout(io.StringIO()) as stdout:
            code = validation.main(['--run-dir', str(self.run), '--raw-dir', str(self.raw),
                '--stop-station-m', '10', '--output', str(self.run/'absent'/'result.json')])
        self.assertEqual(code, 2)
        self.assertEqual(json.loads(stdout.getvalue())['verdict'], 'INCOMPLETE')

    def test_mismatched_raw_session_is_rejected(self):
        self.write(self.raw/'session.json', dict(session_id='another-run'))
        self.assertIn('run_raw_session_mismatch', self.codes(self.evaluate()))

    def test_missing_lap_marker_and_sparse_measurements_cannot_pass(self):
        (self.run/'lap_complete.json').unlink()
        self.assertIn('full_lap_marker_missing', self.codes(self.evaluate(require_full_lap=True)))
        self.ego = self.ego[::4]
        self.save_rows()
        self.assertEqual(self.evaluate()['verdict'], 'INCOMPLETE')

    def test_json_cli_returns_nonzero_for_failed_archive(self):
        self.write(self.raw/'collisions.json', dict(events=[{'t': 18.}], messages=1))
        output = self.run/'validation.json'
        with contextlib.redirect_stdout(io.StringIO()) as stdout:
            code = validation.main(['--run-dir', str(self.run), '--raw-dir', str(self.raw),
                '--stop-station-m', '10', '--output', str(output)])
        self.assertEqual(code, 1)
        self.assertEqual(json.loads(stdout.getvalue())['verdict'], 'FAIL')
        self.assertEqual(json.loads(output.read_text())['verdict'], 'FAIL')


if __name__ == '__main__':
    unittest.main()
