"""Publication-owned launch evidence cannot become a latched gap permission."""
import math
import unittest

import numpy as np

from junction_launch import JunctionLaunchMemory
from ring_entry import LAUNCH_SLOW


class JunctionLaunchTests(unittest.TestCase):
    def setUp(self):
        self.memory = JunctionLaunchMemory()
        self.path = np.column_stack((np.arange(0., 60.01, .5), np.zeros(121)))

    def ego(self, now, x=0., speed=0., **changes):
        value = dict(x=x, y=0., heading_deg=0., speed_kmh=speed*3.6,
                     stamp=1000.+now, age_s=.01, received=now-.01, accel=.4, brake=0.)
        value.update(changes)
        return value

    def publish(self, now, ego=None, path=None, target=6., **changes):
        args = dict(proof_expires_at=now+.05)
        args.update(changes)
        return self.memory.published(self.path if path is None else path,
            self.ego(now) if ego is None else ego, target, now, **args)

    def rolling(self):
        for now, x, speed in ((10., 0., 1.), (10.1, .08, 1.1), (10.2, .17, 1.2)):
            ego = self.ego(now, x, speed)
            self.assertTrue(self.publish(now, ego))
            result = self.memory.prediction(self.path, ego, now, .8)
        return ego, result

    def test_an_unpublished_go_proposal_supplies_no_prediction(self):
        self.assertIsNone(self.memory.prediction(self.path, self.ego(10.), 10., .8))
        self.assertFalse(self.memory.active)

    def test_only_successful_fresh_positive_proofs_can_seed_the_memory(self):
        for changes in (dict(target=0.), dict(emergency=True), dict(proof_valid=False),
                        dict(proof_expires_at=9.9), dict(proof_expires_at=math.nan)):
            self.assertFalse(self.publish(10., **changes))
            self.assertFalse(self.memory.active)
        self.assertFalse(self.publish(10., self.ego(10., speed=1.), target=1.15))

    def test_initial_wait_brake_seeds_without_permission_then_throttle_ack_credits_dead_time(self):
        held = self.ego(10., accel=0., brake=1.)
        self.assertTrue(self.publish(10., held))
        self.assertIsNone(self.memory.prediction(self.path, held, 10., .8))
        self.assertTrue(self.memory.active)
        # Rereading that original pre-actuation sample grants no prediction.
        self.assertIsNone(self.memory.prediction(self.path, held, 10.1, .8))
        self.assertTrue(self.memory.active)
        ack = self.ego(10.15, x=.005, speed=.1)
        result = self.memory.prediction(self.path, ack, 10.15, .8)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result['v_mps'], 0.)
        self.assertAlmostEqual(result['dead_s'], .5-.15*.8)
        self.assertEqual(result['stamp'], ack['stamp'])
        self.assertEqual(set(result), {'v_mps', 'dead_s', 'stamp'})

    def test_initial_brake_cannot_remain_pending_past_the_slow_model_dead_time(self):
        for now in (10., 10.2, 10.4, 10.6):
            held = self.ego(now, accel=0., brake=1.)
            self.assertTrue(self.publish(now, held))
            self.assertIsNone(self.memory.prediction(self.path, held, now, .8))
        held = self.ego(10.65, accel=0., brake=1.)
        self.assertIsNone(self.memory.prediction(self.path, held, 10.65, .8))
        self.assertFalse(self.memory.active)

    def test_recent_actual_progress_uses_a_lower_observed_speed_and_keeps_the_original_model(self):
        ego, result = self.rolling()
        self.assertAlmostEqual(result['v_mps'], .7)
        self.assertLess(result['v_mps'], ego['speed_kmh']/3.6)
        self.assertAlmostEqual(result['dead_s'], .5-.2*.8)
        self.assertEqual(LAUNCH_SLOW, (2.5, .5))

    def test_a_previous_proof_expiring_does_not_erase_recent_confirmed_actuation(self):
        self.assertTrue(self.publish(10., self.ego(10.)))
        later = self.ego(10.12, .005, .1)
        result = self.memory.prediction(self.path, later, 10.12, .8)
        self.assertIsNotNone(result)  # This is a model input, never a GO proof.
        self.assertLess(result['dead_s'], .5)
        self.memory.clear('fresh GP now says WAIT')
        self.assertIsNone(self.memory.prediction(self.path, later, 10.12, .8))

    def test_actual_stop_brake_throttle_release_and_too_low_target_clear_a_rolling_launch(self):
        for changes in (dict(brake=.01), dict(accel=0.), dict(speed_kmh=6.*3.6)):
            self.memory = JunctionLaunchMemory()
            ego, _ = self.rolling()
            changed = self.ego(10.3, .27, 1.2, **changes)
            self.assertIsNone(self.memory.prediction(self.path, changed, 10.3, .8))
            self.assertFalse(self.memory.active)
        self.memory = JunctionLaunchMemory()
        self.rolling()
        self.assertFalse(self.publish(10.25, self.ego(10.25, .22, 1.2), target=0.))
        self.assertFalse(self.memory.active)

    def test_stale_pose_source_rewind_conflicting_stamp_and_implausible_motion_clear(self):
        examples = (self.ego(10.3, .27, 1.3, age_s=.201),
                    self.ego(10.3, .27, 1.3, received=10.),
                    self.ego(10.3, .27, 1.3, stamp=1010.1),
                    self.ego(10.3, .28, 1.2, stamp=1010.2),
                    self.ego(10.3, 3., 1.3), self.ego(10.3, .05, 1.3),
                    self.ego(10.3, .25, .5))
        for ego in examples:
            self.memory = JunctionLaunchMemory()
            self.rolling()
            self.assertIsNone(self.memory.prediction(self.path, ego, 10.3, .8))
            self.assertFalse(self.memory.active)

    def test_duplicate_pose_reads_do_not_manufacture_velocity_or_renew_command_age(self):
        ego = self.ego(10., speed=1.)
        self.assertTrue(self.publish(10., ego))
        for now in (10., 10.03, 10.06, 10.09):
            result = self.memory.prediction(self.path, ego, now, .8)
            self.assertEqual(result['v_mps'], 0.)
        self.assertIsNone(self.memory.prediction(self.path, self.ego(10.351, .25, 1.), 10.351, .8))
        self.assertFalse(self.memory.active)

    def test_trimmed_and_resampled_same_world_curve_keeps_launch_evidence(self):
        angle = np.linspace(0., 1.2, 121)
        curved = np.column_stack((50.*np.sin(angle), 50.*(1.-np.cos(angle))))
        self.assertTrue(self.publish(10., self.ego(10.), path=curved))
        trimmed = curved[2:]
        ego = self.ego(10.1, .08, .1)
        result = self.memory.prediction(trimmed, ego, 10.1, .8)
        self.assertIsNotNone(result)
        self.assertLess(result['dead_s'], .5)

    def test_shifted_or_heading_changed_path_drops_state_including_distant_common_prefix_changes(self):
        paths = []
        shifted = self.path.copy(); shifted[:, 1] += .101; paths.append(shifted)
        distant = self.path.copy(); distant[90:, 1] = .11; paths.append(distant)
        turn = self.path.copy(); turn[40:, 1] = (turn[40:, 0]-20.)*math.tan(math.radians(6.1)); paths.append(turn)
        paths.append(self.path[:10])
        for path in paths:
            self.memory = JunctionLaunchMemory()
            self.assertTrue(self.publish(10.))
            self.assertIsNone(self.memory.prediction(path, self.ego(10.1), 10.1, .8))
            self.assertFalse(self.memory.active)

    def test_rtf_must_be_valid_and_increases_cannot_overcredit_prior_slow_intervals(self):
        self.publish(10.)
        first = self.memory.prediction(self.path, self.ego(10.1, .005, .1), 10.1, .5)
        self.publish(10.1, self.ego(10.1, .005, .1))
        second = self.memory.prediction(self.path, self.ego(10.2, .02, .2), 10.2, .9)
        self.assertAlmostEqual(first['dead_s'], .45)
        self.assertAlmostEqual(second['dead_s'], .4)
        self.assertIsNone(self.memory.prediction(self.path, self.ego(10.25, .03, .2), 10.25, math.nan))
        self.assertFalse(self.memory.active)

    def test_a_new_fresh_launch_after_clear_cannot_reuse_old_velocity_or_dead_credit(self):
        self.rolling()
        self.memory.clear('person or raw stop')
        ego = self.ego(10.3, .2, 0.)
        self.assertTrue(self.publish(10.3, ego))
        result = self.memory.prediction(self.path, ego, 10.3, .8)
        self.assertEqual(result, dict(v_mps=0., dead_s=.5, stamp=ego['stamp']))

    def test_accelerating_approach_observation_ends_when_a_positive_cap_requests_deceleration(self):
        # A safe approach may still be waiting for a traffic gap. The memory
        # measures the physical response; it does not grant gap permission.
        for now, x, speed in ((10., 0., 1.), (10.15, .12, 1.2), (10.3, .27, 1.4)):
            ego = self.ego(now, x, speed)
            self.assertTrue(self.publish(now, ego, target=3.))
            observed = self.memory.prediction(self.path, ego, now, .8)
        self.assertGreater(observed['v_mps'], 0.)
        self.assertLess(observed['v_mps'], ego['speed_kmh']/3.6)
        # A positive braking envelope is not continuous acceleration, even
        # while the native feedback still echoes the preceding throttle.
        self.assertFalse(self.publish(10.35, self.ego(10.35, .32, 1.4), target=1.3))
        self.assertFalse(self.memory.active)
        self.assertIsNone(self.memory.prediction(self.path, self.ego(10.4, .36, 1.3), 10.4, .8))


if __name__ == '__main__':
    unittest.main()
