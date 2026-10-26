import argparse
import json
import random
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import competition_cycle as cc


class LapTest(unittest.TestCase):
    def test_initial_position_near_finish_cannot_be_reported_as_a_lap(self):
        lap = cc.LapProgress(4392, 0)
        self.assertFalse(lap.observe(4350, 1))
        self.assertFalse(lap.observe(20, 4))

    def test_completed_lap_requires_middle_then_end_then_wrapped_start(self):
        lap = cc.LapProgress(4392, 0)
        for idx, t in [(0, 1), (1400, 60), (2200, 110), (3800, 150), (4320, 170)]:
            self.assertFalse(lap.observe(idx, t))
        self.assertTrue(lap.observe(40, 173))

    def test_slow_status_polling_does_not_mark_highway_movement_as_stuck(self):
        lap = cc.LapProgress(4392, 0)
        lap.observe(2300, 1)
        lap.observe(3000, 15)
        self.assertTrue(lap.moving)
        self.assertEqual(lap.last_move, 15)

    def test_backward_localization_jump_does_not_reset_movement_watchdog(self):
        lap = cc.LapProgress(4392, 0)
        lap.observe(2300, 1)
        lap.observe(2350, 2)
        lap.observe(2100, 3)
        self.assertEqual(lap.last_move, 2)


class OwnershipTest(unittest.TestCase):
    def cycle(self):
        with patch.object(cc.ac, 'route_length', return_value=4392):
            return cc.Cycle(argparse.Namespace(max_minutes=10, stuck_seconds=90))

    def test_generated_scene_must_contain_all_requested_regional_vehicles(self):
        cycle = self.cycle()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'EgoNetwork').mkdir()
            network = root / 'network.json'
            network.write_text('{}')
            scene = dict(vehicleList=[dict(UNIQUEID=i) for i in range(14)], spawnPointList=[],
                         objectList=[{}]*8, pedestrianList=[{}]*2)
            path = root / 'test.json'
            path.write_text(json.dumps(scene))
            config = cc.scenario_config(random.Random(1), 1)
            config['missionCounts'] = dict(static=8, dynamic=2)
            with patch.object(cc, 'scenario_config', return_value=config), patch.object(cc, 'STATE', root), \
                 patch.object(cc.ac, 'SCENARIO_DIR', root), patch.object(cc.ac, 'SINGLE_PC_NETWORK', network), \
                 patch.object(cc.ac, 'http', return_value=dict(saved=['test.json'])):
                name, actual_config, actual_scene = cycle.generate(1, random.Random(1))
                self.assertEqual(actual_config['missionTrafficCounts'], dict(roundabout=4, highway=8, tunnel=2))
                self.assertEqual(len(actual_scene['vehicleList']), 14)
                self.assertEqual((root / 'EgoNetwork/test_MN.json').read_text(), '{}')
                scene['vehicleList'] = scene['vehicleList'][:11]
                path.write_text(json.dumps(scene))
                with self.assertRaisesRegex(RuntimeError, 'NPC 14대'):
                    cycle.generate(2, random.Random(1))

    def test_cleanup_never_stops_a_replacement_manual_drive(self):
        cycle = self.cycle()
        cycle.owned_start = 10
        with patch.object(cc.ac, 'job', return_value=dict(running=True, started_at=11)), patch.object(cc.ac, 'http') as http:
            cycle.stop_owned()
            http.assert_not_called()

    def test_cleanup_waits_for_its_own_drive_to_stop(self):
        cycle = self.cycle()
        cycle.owned_start = 10
        with patch.object(cc.ac, 'job', return_value=dict(running=True, started_at=10)), patch.object(cc.ac, 'http') as http, patch.object(cc.ac, 'wait_job_idle', return_value=True) as idle:
            cycle.stop_owned()
            http.assert_called_once_with(cc.ac.WEB+'/api/stop', {}, timeout=30)
            idle.assert_called_once_with(60)

    def test_user_stop_ends_the_cycle_instead_of_starting_the_next_scenario(self):
        cycle = self.cycle()
        jobs = [dict(running=False), dict(running=True, started_at=10), dict(running=False, started_at=10)]
        response = dict(job=dict(running=False, started_at=10, exit_code=130, logs=[]))
        with patch.object(cycle, 'status'), patch.object(cc.ac, 'job', side_effect=jobs), patch.object(cc.ac, 'ensure_network'), patch.object(cc.ac, 'http', side_effect=[{'ok':True}, response]):
            result = cycle.drive()
        self.assertTrue(cycle.stopping)
        self.assertEqual(result['outcome'], 'stopped_by_user')

    def test_long_model_startup_keeps_camera_recorder_heartbeat_alive(self):
        cycle = self.cycle()
        clock = [0.]
        initializing = dict(job=dict(running=True, started_at=10, logs=[]), ros=dict(follower_running=False))
        ended = dict(job=dict(running=False, started_at=10, exit_code=1, logs=[]))
        def sleep(seconds):
            clock[0] += seconds
        with patch.object(cycle, 'status') as status, patch.object(cycle, 'stop_owned'), \
             patch.object(cc.ac, 'job', side_effect=[dict(running=False), dict(running=True, started_at=10)]), \
             patch.object(cc.ac, 'ensure_network'), \
             patch.object(cc.ac, 'http', side_effect=[{'ok': True}]+[initializing]*20+[ended]), \
             patch.object(cc.time, 'sleep', side_effect=sleep), \
             patch.object(cc.time, 'monotonic', side_effect=lambda: clock[0]):
            result = cycle.drive()
        self.assertEqual(clock[0], 40.)
        self.assertEqual(status.call_count, 21)
        self.assertTrue(all(call.kwargs['phase'] == 'starting_drive' for call in status.call_args_list))
        self.assertEqual(result['outcome'], 'startup_error')

    def test_failure_after_movement_is_reported_as_runtime_error(self):
        cycle = self.cycle()
        running = dict(running=True, started_at=10, logs=[])
        responses = [dict(job=running, ros=dict(follower_running=True, path=dict(nearest_idx=idx))) for idx in [0, 30]]
        responses.append(dict(job=dict(running=False, started_at=10, exit_code=1, logs=[])))
        with patch.object(cycle, 'status'), patch.object(cycle, 'stop_owned'), \
             patch.object(cc.ac, 'job', side_effect=[dict(running=False), running]), \
             patch.object(cc.ac, 'ensure_network'), patch.object(cc.time, 'sleep'), \
             patch.object(cc.ac, 'http', side_effect=[{'ok': True}]+responses):
            result = cycle.drive()
        self.assertEqual(result['outcome'], 'runtime_error')
        self.assertEqual(result['max_route_idx'], 30)


if __name__ == '__main__':
    unittest.main()
