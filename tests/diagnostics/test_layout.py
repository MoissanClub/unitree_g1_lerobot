"""Regression checks for package boundaries, assets, and operator entry points."""

import os
import ast
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


class LayoutTests(unittest.TestCase):
    def test_nested_tests_remain_discoverable(self):
        self.assertTrue((ROOT / "tests/__init__.py").is_file())
        for group in ("robots", "simulation", "xr", "diagnostics", "fixtures"):
            self.assertTrue((ROOT / "tests" / group / "__init__.py").is_file())
        for source in (ROOT / "tests").rglob("test_*.py"):
            self.assertNotEqual(source.parent, ROOT / "tests")
            self.assertTrue((source.parent / "__init__.py").is_file())

    def test_dependency_direction(self):
        for source in (ROOT / "unitree_g1_lerobot").rglob("*.py"):
            for node in ast.walk(ast.parse(source.read_text())):
                if isinstance(node, ast.Import):
                    modules = [alias.name for alias in node.names]
                elif isinstance(node, ast.ImportFrom):
                    modules = [node.module or ""]
                else:
                    continue
                for module in modules:
                    self.assertNotEqual(module.split(".")[0], "tests", str(source))
                    if "robots" in source.relative_to(ROOT).parts:
                        self.assertNotIn("diagnostics", module.split("."), str(source))

    def test_package_uses_descriptive_names(self):
        stage_name = re.compile(r"rung|step[ _-]?\d", re.IGNORECASE)
        for source in (ROOT / "unitree_g1_lerobot").rglob("*.py"):
            with self.subTest(source=source.relative_to(ROOT)):
                self.assertIsNone(stage_name.search(source.stem))
                self.assertIsNone(stage_name.search(source.read_text()))

    def test_robot_support_does_not_load_xr_or_simulation(self):
        subprocess.run(
            [sys.executable, "-c", "\n".join([
                "import sys",
                "from unitree_g1_lerobot.robots import control, g1_embodiments",
                "assert g1_embodiments.G1_23_SPEC.urdf_path.is_file()",
                "assert len(g1_embodiments.G1_23_SPEC.arm_joint_names_g1) == 10",
                "assert not any(n.startswith(('isaacteleop', 'unitree_g1_lerobot.xr', 'unitree_g1_lerobot.simulation')) for n in sys.modules)",
            ])],
            cwd=ROOT,
            check=True,
            timeout=30,
        )

    def test_module_entry_points(self):
        for module in (
            "simulation.g1_compare_ik_viewer",
            "simulation.g1_mujoco_dds_sim",
            "diagnostics.simulation.g1_startup_diagnostic",
            "diagnostics.simulation.view_robot_camera",
            "diagnostics.verify_live_control",
            "diagnostics.simulation.compare_motor_configs",
            "diagnostics.simulation.audit_g1_sources",
            "diagnostics.xr.verify_xr_headless",
            "diagnostics.physical.physical_preflight",
            "diagnostics.physical.verify_lerobot_connection",
            "xr.xr_to_g1_mujoco",
            "xr.xr_controller_cloudxr_smoke_test",
        ):
            with self.subTest(module=module):
                result = subprocess.run(
                    [sys.executable, "-m", f"unitree_g1_lerobot.{module}", "--help"],
                    cwd=ROOT, capture_output=True, text=True, timeout=30,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("usage:", result.stdout)

    def test_shell_syntax(self):
        for launcher in ROOT.glob("*.sh"):
            with self.subTest(launcher=launcher.name):
                subprocess.run(["bash", "-n", str(launcher)], check=True)

    def test_operator_launchers_from_another_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            env = dict(os.environ, SKIP_G1_STARTUP_DIAGNOSTIC="1")
            for name in (
                "run_compare_g1_29_g1_23_ik.sh",
                "run_compare_g1_29_g1_23_urdf_mesh.sh",
                "run_g1_mujoco_dds_sim.sh",
                "run_xr_g1_mujoco.sh",
                "view_g1_camera.sh",
                "run_verify_live_control.sh",
                "run_g1_physical_preflight.sh",
                "verify_g1_lerobot_connection.sh",
                "run_compare_g1_29_motor_configs_no_gravity_compensation.sh",
                "compare_motor_config.sh",
                "run_compare_g1_29_motor_configs_with_gravity_compensation.sh",
                "run_compare_g1_29_g1_23_motor_configs_with_gravity_compensation.sh",
                "run_compare_g1_29_g1_23_motor_configs_no_gravity_compensation.sh",
            ):
                with self.subTest(launcher=name):
                    result = subprocess.run(
                        [str(ROOT / name), "--help"], cwd=temporary, env=env,
                        capture_output=True, text=True, timeout=30,
                    )
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertIn("usage:", result.stdout)

    def test_cloudxr_launcher_resolves_moved_config_from_another_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            venv = Path(temporary) / "venv"
            (venv / "bin").mkdir(parents=True)
            stub = venv / "bin" / "python"
            stub.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
            stub.chmod(0o755)
            env = os.environ.copy()
            env.pop("CLOUDXR_ENV_FILE", None)
            request, ack = Path(temporary) / "request", Path(temporary) / "ack"
            request.write_text("untouched request")
            ack.write_text("untouched ack")
            env.pop("SKIP_G1_STARTUP_DIAGNOSTIC", None)
            env.update(VENV_DIR=str(venv), LEROBOT_ROOT="/nonexistent/lerobot",
                       G1_PYTHON_BIN="/nonexistent/python", G1_DIAGNOSTIC_REQUEST_FILE=str(request),
                       G1_DIAGNOSTIC_ACK_FILE=str(ack))
            result = subprocess.run(
                [str(ROOT / "run_isaac_teleop.sh")], cwd=temporary, env=env,
                capture_output=True, text=True, timeout=15,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("unitree_g1_lerobot.xr.cloudxr_session", result.stdout)
            config = ROOT / "configs/cloudxr_quest3.env"
            self.assertTrue(config.is_file())
            self.assertIn(str(config), result.stdout)
            self.assertNotIn("--embodiment", result.stdout)
            self.assertEqual(request.read_text(), "untouched request")
            self.assertEqual(ack.read_text(), "untouched ack")

    def test_cloudxr_rejects_removed_robot_options(self):
        for option in ("--embodiment", "--skip-startup-diagnostic"):
            result = subprocess.run([str(ROOT / "run_isaac_teleop.sh"), option],
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 2)
            self.assertIn("Unknown argument", result.stderr)


if __name__ == "__main__":
    unittest.main()
