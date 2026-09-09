"""Configuration and transport-index contracts; these tests do not connect to DDS."""
import unittest
from unittest.mock import Mock

import numpy as np

from unitree_g1_lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
from lerobot.robots.unitree_g1.config_unitree_g1 import _DEFAULT_KP, _DEFAULT_KD
from lerobot.robots.unitree_g1.unitree_g1 import G1_29_LowState
from unitree_g1_lerobot.robots.motor_configs import load_profile


class G1RuntimeTests(unittest.TestCase):
    def test_default_is_unchanged_g1_29(self):
        config = UnitreeG1Config()
        self.assertEqual(config.embodiment, "g1_29")
        self.assertEqual(config.kp, _DEFAULT_KP)
        self.assertEqual(config.kd, _DEFAULT_KD)
        other = UnitreeG1Config()
        config.kp[0] = 0
        self.assertNotEqual(config.kp, other.kp)

    def test_g1_23_gains_are_sparse_and_source_derived(self):
        config = UnitreeG1Config(embodiment="g1_23")
        for motor in load_profile("g1_23")["motors"]:
            self.assertEqual(config.kp[motor["dds_index"]], motor["kp"])
        for i in (13, 14, 20, 21, 27, 28):
            self.assertEqual(config.kp[i], 0)
            self.assertEqual(config.kd[i], 0)

    def test_invalid_config_rejected(self):
        for kwargs in ({"embodiment": "g1_22"}, {"kp": [1] * 23}, {"control_dt": 0},
                       {"embodiment": "g1_23", "controller": "GrootLocomotionController"},
                       {"embodiment": "g1_23", "kp": [1] * 29}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                UnitreeG1Config(**kwargs)

    def test_same_class_variant_features_and_feedback(self):
        for variant, count in (("g1_29", 29), ("g1_23", 23)):
            robot = UnitreeG1(UnitreeG1Config(embodiment=variant))
            self.assertEqual(len(robot.action_features), count)
            state = G1_29_LowState()
            for i, motor in enumerate(state.motor_state):
                motor.q = i / 10
            robot._lowstate = state
            obs = robot.get_observation()
            self.assertEqual(sum(k.endswith(".q") for k in obs), count)
            self.assertEqual(obs["kRightShoulderPitch.q"], 2.2)
            if variant == "g1_23":
                self.assertNotIn("kLeftWristPitch.q", obs)

    def test_sparse_feedforward_scatter(self):
        robot = UnitreeG1(UnitreeG1Config(embodiment="g1_23"))
        robot.config.gravity_compensation = True
        robot.arm_ik = Mock()
        robot.arm_ik.solve_tau.return_value = np.arange(10) + 1
        robot.publish_lowcmd = Mock()
        action = {f"{j.name}.q": j.value / 100 for j in robot.arm_index}
        # send_action also reads the held target when an action omits a joint.
        robot.msg = Mock()
        robot.msg.motor_cmd = [Mock(q=0.0) for _ in range(35)]
        self.assertEqual(robot.send_action(action), action)
        tau = robot.publish_lowcmd.call_args.kwargs["tau"]
        np.testing.assert_array_equal(tau[[15,16,17,18,19,22,23,24,25,26]], np.arange(10) + 1)
        np.testing.assert_array_equal(tau[[13,14,20,21,27,28]], 0)

    def test_unintegrated_backend_fails_before_transport(self):
        for simulation in (True, False):
            robot = UnitreeG1(UnitreeG1Config(embodiment="g1_23", is_simulation=simulation))
            robot._ChannelFactoryInitialize = Mock()
            with self.assertRaises(NotImplementedError):
                robot.connect()
            robot._ChannelFactoryInitialize.assert_not_called()

    def test_config_round_trip_and_factory(self):
        import draccus
        from lerobot.robots.utils import make_robot_from_config
        for variant in ("g1_29", "g1_23"):
            config = UnitreeG1Config(embodiment=variant)
            decoded = draccus.decode(UnitreeG1Config, draccus.encode(config))
            self.assertEqual(config, decoded)
            robot = make_robot_from_config(decoded)
            self.assertIs(type(robot), UnitreeG1)
            self.assertEqual(robot.embodiment.name, variant)

    def test_publisher_writes_only_active_dds_slots(self):
        for variant in ("g1_29", "g1_23"):
            robot = UnitreeG1(UnitreeG1Config(embodiment=variant))
            robot.msg = Mock()
            robot.msg.motor_cmd = [Mock(q=-999, tau=-999) for _ in range(35)]
            robot.kp, robot.kd = robot.config.kp, robot.config.kd
            robot.crc, robot.lowcmd_publisher = Mock(), Mock()
            action = {f"{j.name}.q": j.value / 100 for j in robot.joint_index}
            robot.publish_lowcmd(action)
            active = {int(j) for j in robot.joint_index}
            for i in range(35):
                self.assertEqual(robot.msg.motor_cmd[i].q, i / 100 if i in active else -999)
            robot.lowcmd_publisher.Write.assert_called_once_with(robot.msg)

    def test_real_g1_23_gravity_adapter_order(self):
        from unitree_g1_lerobot.robots.g1_embodiments import make_arm_ik
        ik = make_arm_ik("g1_23")
        q = np.linspace(-0.05, 0.15, 10)
        expected = ik._pin.rnea(ik.reduced_robot.model, ik.reduced_robot.model.createData(),
                               q[ik._arm_reorder_g1_to_pin], np.zeros(10), np.zeros(10))
        np.testing.assert_allclose(ik.solve_tau(q), expected[ik._arm_reorder_pin_to_g1])
        with self.assertRaises(ValueError):
            ik.solve_tau(np.zeros(14))
        with self.assertRaises(ValueError):
            ik.solve_tau(np.full(10, np.nan))


if __name__ == "__main__":
    unittest.main()
