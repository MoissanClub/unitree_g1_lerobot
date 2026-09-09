"""Physics and source-derivation checks for the arm gain comparison."""
import unittest

import numpy as np
import mujoco

from unitree_g1_lerobot.diagnostics.motor_suite import SUITE, ready, target
from unitree_g1_lerobot.robots.motor_configs import derive_profile, load_profile, lerobot_profile
from unitree_g1_lerobot.simulation.motor_bench import MotorPlant, mesh_directory, run_trial


class MotorConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.meshes = mesh_directory()

    def test_profiles_match_pinned_sources_and_exclude_placeholders(self):
        for variant, count, arms in (("g1_29", 29, 14), ("g1_23", 23, 10)):
            profile = load_profile(variant)
            self.assertEqual(profile, derive_profile(variant))
            self.assertEqual(len(profile["motors"]), count)
            self.assertEqual(sum(m["arm"] for m in profile["motors"]), arms)
            self.assertEqual(len({m["dds_index"] for m in profile["motors"]}), count)
        indices = {m["dds_index"] for m in load_profile("g1_23")["motors"]}
        self.assertTrue(indices.isdisjoint({13, 14, 20, 21, 27, 28}))

    def test_gain_ab_uses_identical_physics(self):
        a = MotorPlant(load_profile("g1_29"), self.meshes)
        b = MotorPlant(lerobot_profile(), self.meshes)
        for attribute in ("body_mass", "body_inertia", "jnt_range", "dof_damping", "dof_armature", "dof_frictionloss", "actuator_ctrlrange"):
            np.testing.assert_array_equal(getattr(a.model, attribute), getattr(b.model, attribute))
        np.testing.assert_array_equal(a.kd, b.kd)
        self.assertGreater(a.kp[a.joints.index("left_shoulder_pitch_joint")], b.kp[b.joints.index("left_shoulder_pitch_joint")])

    def test_commands_match_on_common_joints_and_respect_limits(self):
        plants = [MotorPlant(load_profile(v), self.meshes) for v in ("g1_29", "g1_23")]
        common = plants[1].joints
        for test in SUITE:
            for t in np.linspace(0, test.seconds, 50):
                commands = [target(test, t, p.joints) for p in plants]
                for p, command in zip(plants, commands):
                    self.assertTrue(np.all(command >= p.ranges[:, 0]))
                    self.assertTrue(np.all(command <= p.ranges[:, 1]))
                np.testing.assert_array_equal(commands[0][[plants[0].joints.index(j) for j in common]], commands[1])

    def test_motor_torque_is_clipped_and_does_not_teleport_joints(self):
        plant = MotorPlant(load_profile("g1_23"), self.meshes)
        command = ready(plant.joints) + 1
        raw, applied = plant.step(command)
        self.assertTrue(np.any(np.abs(raw) > plant.limits))
        self.assertTrue(np.all(np.abs(applied) <= plant.limits))
        self.assertGreater(np.max(np.abs(plant.data.qpos[plant.qadr] - command)), 0.5)

    def test_hold_has_gravity_error_and_repeats_deterministically(self):
        plant = MotorPlant(load_profile("g1_23"), self.meshes)
        a = run_trial(plant, SUITE[0])
        b = run_trial(plant, SUITE[0])
        np.testing.assert_array_equal(a["log"]["q"], b["log"]["q"])
        self.assertGreater(a["metrics"]["joint_rmse_rad"], 0.001)
        self.assertLess(a["metrics"]["joint_rmse_rad"], 0.1)
        self.assertEqual(a["metrics"]["max_limit_violation_rad"], 0)

    def test_gravity_compensation_holds_both_models_with_known_payload(self):
        for profile in (load_profile("g1_29"), lerobot_profile(), load_profile("g1_23")):
            for payload in (0, 0.5):
                plant = MotorPlant(profile, self.meshes, payload, gravity_compensation=True)
                result = run_trial(plant, SUITE[0])
                self.assertLess(result["metrics"]["joint_rmse_rad"], 1e-6)
                self.assertGreater(np.max(np.abs(result["log"]["gravity_feedforward"])), 0.1)
                self.assertEqual(result["metrics"]["saturation_fraction"], 0)

    def test_feedforward_is_gravity_only_and_total_torque_is_clipped(self):
        for variant in ("g1_29", "g1_23"):
            plant = MotorPlant(load_profile(variant), self.meshes, gravity_compensation=True)
            for fraction in (0.2, 0.5, 0.8):
                plant.data.qpos[plant.qadr] = plant.ranges[:, 0] + fraction * np.diff(plant.ranges, axis=1).ravel()
                plant.data.qvel[:] = 0
                mujoco.mj_forward(plant.model, plant.data)
                expected = plant.data.qfrc_bias[plant.vadr].copy()
                plant.data.qvel[:] = 2
                np.testing.assert_allclose(plant.gravity_torque(), expected, atol=1e-12)
                np.testing.assert_array_equal(plant.data.qvel, 2)
            plant.reset()
            command = ready(plant.joints) + 1
            expected = plant.kp * (command - plant.data.qpos[plant.qadr]) + plant.gravity_torque()
            raw, applied = plant.step(command)
            np.testing.assert_allclose(raw, expected)
            np.testing.assert_allclose(applied, np.clip(expected, -plant.limits, plant.limits))

    def test_extended_hold_exposes_sag_without_saturation(self):
        test = next(t for t in SUITE if t.name == "extended_hold_1kg")
        for variant in ("g1_29", "g1_23"):
            plant = MotorPlant(load_profile(variant), self.meshes, test.payload_kg)
            plant.reset(target(test, 0, plant.joints))
            self.assertEqual(plant.data.ncon, 0)
            results = [run_trial(MotorPlant(load_profile(variant), self.meshes, test.payload_kg,
                                           gravity_compensation=mode), test) for mode in (False, True)]
            self.assertGreater(results[0]["metrics"]["last_second_hand_sag_m"], 0.01)
            self.assertLess(abs(results[1]["metrics"]["last_second_hand_sag_m"]), 1e-5)
            for result in results:
                self.assertLess(result["metrics"]["peak_gravity_torque_fraction"], 0.8)
                self.assertEqual(result["metrics"]["saturation_fraction"], 0)

    def test_fast_step_metrics_and_reversal_endpoints(self):
        test = next(t for t in SUITE if t.name == "pitch_20deg_step")
        plant = MotorPlant(load_profile("g1_29"), self.meshes, gravity_compensation=True)
        metrics = run_trial(plant, test)["metrics"]
        self.assertGreater(metrics["rise_to_90_s"], 0)
        self.assertGreater(metrics["rise_10_to_90_s"], 0)
        self.assertGreater(metrics["max_step_overshoot_pct"], 0)
        self.assertIsNotNone(metrics["settling_to_target_s"])
        for test in SUITE:
            if test.kind == "reversal":
                np.testing.assert_allclose(target(test, test.seconds, plant.joints), ready(plant.joints))

    def test_summary_panel_order(self):
        from unitree_g1_lerobot.diagnostics.compare_motor_configs import comparison_cases
        cases = comparison_cases("summary")
        self.assertEqual([mode for _, mode in cases], [False] * 3 + [True] * 3)
        self.assertEqual([p["name"] for p, _ in cases[:3]],
                         [lerobot_profile()["name"], load_profile("g1_29")["name"], load_profile("g1_23")["name"]])
        self.assertEqual(cases[:3], [(p, False) for p, _ in cases[3:]])


if __name__ == "__main__":
    unittest.main()
