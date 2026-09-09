"""Physics and source-derivation checks for the arm gain comparison."""
import unittest

import numpy as np

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


if __name__ == "__main__":
    unittest.main()
