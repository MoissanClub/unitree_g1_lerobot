"""Real loopback DDS checks; run with G1_TEST_DDS=1 on an idle simulator session."""
import os
from functools import wraps
from pathlib import Path
import subprocess
import sys
import time
import unittest

from unitree_g1_lerobot.simulation.dds import (
    connect_unitree_g1_external_dds, patch_unitree_dds_config, verify_simulator_identity,
)


def isolated_dds(test):
    @wraps(test)
    def run(self):
        if os.environ.get("G1_DDS_TEST_CHILD") == test.__name__:
            return test(self)
        # SDK publication-matched callbacks can outlive Python listeners during
        # GC between sessions. Match the real launchers' process isolation.
        root = Path(__file__).resolve().parents[1]
        env = dict(os.environ, G1_DDS_TEST_CHILD=test.__name__,
                   PYTHONPATH=str(root) + os.pathsep + os.environ.get("PYTHONPATH", ""))
        result = subprocess.run(
            [sys.executable, "-m", "unittest", f"test_native_g1_dds.NativeG1DDSTests.{test.__name__}", "-v"],
            cwd=root / "tests", env=env, capture_output=True, text=True, timeout=45,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
    return run


@unittest.skipUnless(os.environ.get("G1_TEST_DDS") == "1", "requires an idle loopback DDS session")
class NativeG1DDSTests(unittest.TestCase):
    @isolated_dds
    def test_standalone_command_and_feedback(self):
        from unitree_g1_lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
        from lerobot.robots.unitree_g1 import unitree_g1 as g1_module
        root = Path(__file__).resolve().parents[1]
        process = subprocess.Popen([str(root / "run_g1_mujoco_dds_sim.sh"), "--embodiment", "g1_23",
                                    "--no-view", "--skip-startup-diagnostic", "--duration-s", "12"],
                                   cwd=root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        robot = None
        try:
            patch_unitree_dds_config()
            robot = UnitreeG1(UnitreeG1Config(embodiment="g1_23", gravity_compensation=True))
            connect_unitree_g1_external_dds(robot, g1_module, robot.joint_index, 8)
            with self.assertRaisesRegex(ValueError, "mismatch"):
                verify_simulator_identity("g1_29", 1)
            initial = robot.get_observation()
            self.assertEqual(sum(k.endswith(".q") for k in initial), 23)
            action = {k: v for k, v in initial.items() if k.endswith(".q")}
            for side in ("Left", "Right"):
                action[f"k{side}ShoulderPitch.q"] = -0.85
                action[f"k{side}Elbow.q"] = 1.1
            for _ in range(80):
                robot.send_action(action)
                time.sleep(0.02)
            measured = robot.get_observation()
            for side in ("Left", "Right"):
                key = f"k{side}ShoulderPitch.q"
                self.assertLess(measured[key], initial[key] - 0.2)
            time.sleep(0.8)
            held = robot.get_observation()
            time.sleep(0.4)
            later = robot.get_observation()
            self.assertLess(abs(later["kLeftShoulderPitch.q"] - held["kLeftShoulderPitch.q"]), 0.03)
            self.assertEqual(later["kWaistYaw.q"], 0)
        finally:
            if robot is not None:
                robot.disconnect()
            if process.poll() is None:
                process.send_signal(2)
            output, _ = process.communicate(timeout=10)
        self.assertEqual(process.returncode, 0, output)

    @isolated_dds
    def test_embedded_backend_lifecycle(self):
        from unitree_g1_lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
        patch_unitree_dds_config()
        robot = UnitreeG1(UnitreeG1Config(embodiment="g1_23"))
        try:
            robot.connect()
            self.assertTrue(robot.is_connected)
            self.assertEqual(robot.sim_env.embodiment, "g1_23")
            self.assertEqual(len(robot.sim_env.plant.joints), 10)
            self.assertGreater(robot.sim_env.steps, 0)
        finally:
            robot.disconnect()
        self.assertFalse(robot.subscribe_thread.is_alive())


if __name__ == "__main__":
    unittest.main()
