"""Keep legacy NPC fixtures reproducible and opt into rear-axle body geometry."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

import virtual_sim as sim


class VirtualNpcFootprintTests(unittest.TestCase):
    def ego(self, x=-98., y=321.4, yaw_deg=67.9, speed=0., mode="ego_body"):
        return SimpleNamespace(x=x, y=y, yaw=math.radians(yaw_deg), v=speed,
                               npc_reaction_geometry=mode)

    def actor(self, xy=(-106.42443414159953, 326.0618786640686),
              yaw_deg=-11.344507944141515, speed=7. / 3.6 * .6, cruise=None):
        actor = sim.Actor("circ0", "npc", xy, yaw_deg, 5., 2.1, speed=speed)
        if cruise is not None:
            actor.cruise = cruise
        return actor

    def runtime_stub_sim(self, **options):
        route = np.column_stack((np.arange(5.), np.zeros(5)))
        world = dict(waypoints=route, smooth=route, lanes=SimpleNamespace())
        runtime = SimpleNamespace(r=SimpleNamespace(), clock=sim.Clock())
        with patch.object(sim, "SimulatedRuntime", return_value=runtime):
            return sim.Sim(world, 0, duration_s=0., **options)

    def test_default_and_explicit_modes_are_recorded_without_running_a_planner(self):
        default = self.runtime_stub_sim()
        self.assertEqual(default.npc_reaction_geometry, "legacy_point")
        self.assertEqual(default.run()["npc_reaction_geometry"], "legacy_point")
        body = self.runtime_stub_sim(npc_reaction_geometry="ego_body")
        self.assertEqual(body.run()["npc_reaction_geometry"], "ego_body")

    def test_invalid_mode_is_rejected_before_runtime_construction(self):
        with patch.object(sim, "SimulatedRuntime") as runtime:
            with self.assertRaisesRegex(ValueError, "npc_reaction_geometry"):
                sim.Sim({}, 0, npc_reaction_geometry="body-ish")
        runtime.assert_not_called()

    def test_virtual_runtime_constructs_unarmed_production_junction_recovery(self):
        from junction_recovery import JunctionRecovery
        route = np.column_stack((np.arange(5.), np.zeros(5)))
        runtime = sim.SimulatedRuntime(route, route, SimpleNamespace(links=[]),
                                       highway=False, stop_line=False, tunnel=False)
        self.assertIsInstance(runtime.r.junction_recovery, JunctionRecovery)
        self.assertFalse(runtime.r.junction_recovery.seen_start)
        self.assertFalse(runtime.r.junction_recovery.active)
        self.assertFalse(runtime.r.gap_planner.junction_committed)

    def test_legacy_tilted_case362_preserves_the_original_no_braking_result(self):
        ego = self.ego(mode="legacy_point")
        actor = self.actor()
        initial = actor.speed
        sim.Sim.npc_follow_ego(ego, actor)
        self.assertEqual(actor.speed, initial)
        # Lightweight callers that predate the option retain the same model.
        del ego.npc_reaction_geometry
        sim.Sim.npc_follow_ego(ego, actor)
        self.assertEqual(actor.speed, initial)

    def test_legacy_collinear_headway_retains_its_previous_speed(self):
        ego = self.ego(x=20., y=0., yaw_deg=0., speed=3., mode="legacy_point")
        actor = self.actor(xy=(0., 0.), yaw_deg=0., speed=5., cruise=8.)
        sim.Sim.npc_follow_ego(ego, actor)
        self.assertAlmostEqual(actor.speed, 5.09125, places=12)

    def test_case362_body_is_seen_despite_rear_axle_outside_the_lane_band(self):
        ego = self.ego()
        actor = self.actor()
        delta = np.array([ego.x, ego.y]) - actor.xy
        lateral = -delta[0] * math.sin(actor.yaw) + delta[1] * math.cos(actor.yaw)
        self.assertAlmostEqual(lateral, -2.913643363188797)
        self.assertGreater(abs(lateral), 2.3)
        self.assertAlmostEqual(sim.npc_ego_body_headway(ego, actor), 5.60005733260474)
        prior = actor.speed
        sim.Sim.npc_follow_ego(ego, actor)
        self.assertAlmostEqual(actor.speed, prior - 4. * sim.DT)

    def test_case367_requires_full_body_even_when_shifted_centre_misses(self):
        ego = self.ego()
        actor = self.actor(xy=(-103.35391066709771, 325.576597513154),
                           yaw_deg=-3.3770296864827447, speed=7. / 3.6 * 1.5)
        centre = np.mean(sim.ego_body_corners(ego.x, ego.y, ego.yaw), axis=0)
        delta = centre - actor.xy
        lateral = -delta[0] * math.sin(actor.yaw) + delta[1] * math.cos(actor.yaw)
        self.assertAlmostEqual(lateral, -2.407299352919678)
        self.assertGreater(abs(lateral), 2.3)
        self.assertAlmostEqual(sim.npc_ego_body_headway(ego, actor), 1.9411175849444264)
        prior = actor.speed
        sim.Sim.npc_follow_ego(ego, actor)
        self.assertAlmostEqual(actor.speed, prior - 4. * sim.DT)

    def test_headway_uses_actual_rear_face_on_a_straight_lane(self):
        ego = self.ego(x=20., y=0., yaw_deg=0.)
        actor = self.actor(xy=(0., 0.), yaw_deg=0.)
        self.assertAlmostEqual(sim.npc_ego_body_headway(ego, actor), 20. - .79 - 2.5)

    def test_joint_world_rotation_preserves_body_headway(self):
        ego, actor = self.ego(), self.actor()
        expected = sim.npc_ego_body_headway(ego, actor)
        rotated_ego = self.ego(x=-ego.y, y=ego.x, yaw_deg=157.9)
        rotated_actor = self.actor(xy=(-actor.xy[1], actor.xy[0]),
                                   yaw_deg=math.degrees(actor.yaw) + 90.)
        self.assertAlmostEqual(sim.npc_ego_body_headway(rotated_ego, rotated_actor), expected)

    def test_nonoverlapping_side_and_entirely_behind_bodies_do_not_brake(self):
        for x, y in ((20., 4.), (-10., 0.)):
            with self.subTest(x=x, y=y):
                ego = self.ego(x=x, y=y, yaw_deg=0.)
                actor = self.actor(xy=(0., 0.), yaw_deg=0., speed=5., cruise=8.)
                self.assertIsNone(sim.npc_ego_body_headway(ego, actor))
                sim.Sim.npc_follow_ego(ego, actor)
                self.assertAlmostEqual(actor.speed, 5. + 1.5 * sim.DT)

    def test_current_body_contact_brakes_even_if_rear_axle_is_behind_npc(self):
        ego = self.ego(x=-1., y=0., yaw_deg=0.)
        actor = self.actor(xy=(0., 0.), yaw_deg=0., speed=2.)
        self.assertLess(sim.npc_ego_body_headway(ego, actor), 0.)
        sim.Sim.npc_follow_ego(ego, actor)
        self.assertAlmostEqual(actor.speed, 2. - 4. * sim.DT)

    def test_collision_box_wrapper_cannot_double_shift_the_npc_reaction_body(self):
        ego, actor = self.ego(), self.actor()
        with patch.object(sim, "box_corners", side_effect=AssertionError("collision wrapper")):
            self.assertAlmostEqual(sim.npc_ego_body_headway(ego, actor), 5.60005733260474)

    def test_malformed_body_geometry_cannot_silently_become_a_clear_scene(self):
        actor = self.actor(xy=(math.nan, 0.))
        with self.assertRaisesRegex(ValueError, "finite poses"):
            sim.npc_ego_body_headway(self.ego(), actor)


if __name__ == "__main__":
    unittest.main()
