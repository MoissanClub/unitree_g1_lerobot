"""XR selection and diagnostic contracts without live DDS or a headset."""
import argparse
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

from unitree_g1_lerobot.robots.unitree_g1 import get_g1_embodiment
from unitree_g1_lerobot.diagnostics.g1_startup_diagnostic import build_actions
from unitree_g1_lerobot.diagnostics.requests import StartupDiagnosticRequests
from unitree_g1_lerobot.xr.xr_to_g1_mujoco import parse_args


class XREmbodimentTests(unittest.TestCase):
    def test_headless_selection_keeps_real_xr(self):
        for name in ("g1_29", "g1_23"):
            with patch("sys.argv", ["bridge", "--headless", "--embodiment", name]):
                args = parse_args()
            self.assertEqual(args.embodiment, name)
            self.assertTrue(args.no_wait)
            self.assertFalse(args.mock_xr)
            self.assertTrue(args.gravity_compensation)

    def test_gravity_defaults_and_explicit_opt_out(self):
        from unitree_g1_lerobot.simulation.g1_mujoco_dds_sim import parse_args as sim_args
        from unitree_g1_lerobot.diagnostics.g1_startup_diagnostic import parse_args as diagnostic_args
        for parse, prefix in ((parse_args, []), (sim_args, []), (diagnostic_args, ["raise"])):
            for embodiment in ("g1_29", "g1_23"):
                for flags, expected in (([], True), (["--no-gravity-compensation"], False),
                                        (["--gravity-compensation"], True)):
                    with patch("sys.argv", ["test", *prefix, "--embodiment", embodiment, *flags]):
                        self.assertEqual(parse().gravity_compensation, expected)

    def test_simulator_forwards_diagnostic_compensation(self):
        from unitree_g1_lerobot.simulation.g1_mujoco_dds_sim import parse_args as sim_args, run_raise_arm_diagnostic
        for enabled in (True, False):
            with patch("sys.argv", ["sim", "--headless"]):
                args = sim_args()
            args.gravity_compensation = enabled
            with patch("unitree_g1_lerobot.diagnostics.g1_startup_diagnostic.run_diagnostic", return_value=0) as diagnostic:
                self.assertEqual(run_raise_arm_diagnostic(Mock(), args, drive_steps=False), 0)
            self.assertEqual(diagnostic.call_args.args[0].gravity_compensation, enabled)
            self.assertEqual(diagnostic.call_args.args[0].embodiment, "g1_29")

    def test_hands_up_uses_named_wrist_slots(self):
        args = argparse.Namespace(ready_x_m=.03, ready_z_m=.12, ready_spread_m=.03,
                                  orientation_deg=35, hands_up_deg=90, hands_up_direction="inward")
        for name, count in (("g1_29", 29), ("g1_23", 23)):
            spec = get_g1_embodiment(name)
            actions = build_actions(args, spec.make_ik(), spec.arm_index, spec.joint_index)
            self.assertEqual(len(actions["hands-up"]), count)
            self.assertAlmostEqual(actions["hands-up"]["kLeftWristRoll.q"], -np.pi/2)
            self.assertAlmostEqual(actions["hands-up"]["kRightWristRoll.q"], np.pi/2)
            for action in actions.values():
                self.assertTrue(np.isfinite(list(action.values())).all())
            if name == "g1_23":
                self.assertNotIn("kRightWristPitch.q", actions["hands-up"])

    def test_lower_ack_follows_motion_and_checks_embodiment(self):
        with tempfile.TemporaryDirectory() as folder:
            request, ack = Path(folder)/"request", Path(folder)/"ack"
            service = StartupDiagnosticRequests(request, ack, {"up": 1}, {"down": 1}, 30, "g1_23")
            robot = Mock()
            request.write_text(json.dumps({"id": "test", "mode": "lower_hold", "embodiment": "g1_29"}))
            with self.assertRaises(ValueError):
                service.step(robot)
            robot.send_action.assert_not_called()
            self.assertFalse(ack.exists())
            request.write_text(json.dumps({"id": "test", "mode": "lower_hold", "embodiment": "g1_23"}))
            def published(*_):
                self.assertFalse(ack.exists())
            with patch("unitree_g1_lerobot.diagnostics.requests.publish_ready_for", side_effect=published) as publish:
                self.assertTrue(service.step(robot))
            self.assertEqual(publish.call_count, 2)
            self.assertEqual(publish.call_args.args[1], {"down": 1})
            self.assertEqual(json.loads(ack.read_text())["id"], "test")


if __name__ == "__main__":
    unittest.main()
