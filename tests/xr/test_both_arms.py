"""Dual-controller isolation, joint-order, and real-IK regression checks."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np

from unitree_g1_lerobot.robots.unitree_g1 import get_g1_embodiment
from unitree_g1_lerobot.robots.control import fk, make_ready_targets, solve_ready_q
from unitree_g1_lerobot.xr.xr_to_g1_mujoco import ArmTargets, parse_args

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT.parent / "lerobot"))


def frame(pos=(0., 0., 0.), squeeze=.8, quat=(0., 0., 0., 1.)):
    return dict(grip_pos=np.array(pos, dtype=float), grip_quat=np.array(quat, dtype=float),
                squeeze=squeeze, trigger=0.)


class BothArmsTests(unittest.TestCase):
    def test_default_and_single_hand_compatibility(self):
        for side in ("both", "left", "right"):
            with patch("sys.argv", ["bridge", "--hand-side", side]):
                self.assertEqual(parse_args().hand_side, side)
        with patch("sys.argv", ["bridge"]):
            self.assertEqual(parse_args().hand_side, "both")

    def test_real_ik_both_embodiments_and_independent_freeze(self):
        with patch("sys.argv", ["bridge"]):
            args = parse_args()
        for embodiment in ("g1_29", "g1_23"):
            with self.subTest(embodiment=embodiment):
                spec = get_g1_embodiment(embodiment)
                ik = spec.make_ik()
                model = ik.reduced_robot.model
                data = model.createData()
                ik.reduced_robot.data = data
                homes = fk(model, data, ik.L_hand_id, ik.R_hand_id, np.zeros(model.nq))
                left, right = make_ready_targets(*homes, .03, .12, .03)
                q = solve_ready_q(ik, left, right, np.zeros(model.nq))
                control = ArmTargets(ik, spec.arm_index, dict(left=left, right=right), args)
                start = q.copy()
                actions = dict(left=frame(), right=frame())
                tracked = dict(left=True, right=True)
                q, _ = control.update(actions, tracked, q)
                with patch.object(ik, "solve_ik", wraps=ik.solve_ik) as solve:
                    for i in range(1, 61):
                        actions = dict(left=frame((.02*i/60, 0, .02*i/60)),
                                       right=frame((0, -.02*i/60, .03*i/60)))
                        q, status = control.update(actions, tracked, q)
                    self.assertEqual(solve.call_count, 60)
                for side in ("left", "right"):
                    self.assertTrue(status[side]["engaged"])
                    self.assertGreater(np.max(np.abs((q-start)[control.indices[side]])), .01)
                before = {side: pose.copy() for side, pose in control.targets.items()}
                for side, angle in (("left", .08), ("right", -.06)):
                    actions[side]["grip_quat"] = np.array([np.sin(angle/2), 0., 0., np.cos(angle/2)])
                q, _ = control.update(actions, tracked, q)
                for side in before:
                    self.assertGreater(np.linalg.norm(control.targets[side][:3, :3] - before[side][:3, :3]), .01)
                # A released/invalid/untracked hand must stay exactly at its last command.
                for frozen in ("left", "right"):
                    moving = "right" if frozen == "left" else "left"
                    for mode in ("release", "missing", "zero-quat", "nan-pos", "nan-button"):
                        actions[frozen] = frame()
                        tracked[frozen] = mode != "missing"
                        if mode == "release":
                            actions[frozen]["squeeze"] = 0.
                        elif mode == "zero-quat":
                            actions[frozen]["grip_quat"] = np.zeros(4)
                        elif mode == "nan-pos":
                            actions[frozen]["grip_pos"][0] = np.nan
                        elif mode == "nan-button":
                            actions[frozen]["squeeze"] = np.nan
                        tracked[moving] = True
                        actions[moving] = frame()
                        held = q.copy()
                        q, status = control.update(actions, tracked, q)
                        for i in range(1, 16):
                            actions[moving] = frame((.001*i, 0., .001*i))
                            q, status = control.update(actions, tracked, q)
                        np.testing.assert_array_equal(q[control.indices[frozen]], held[control.indices[frozen]])
                        self.assertFalse(status[frozen]["engaged"])
                        self.assertTrue(status[moving]["engaged"])
                        self.assertTrue(np.isfinite(q).all())
                # Re-engagement latches the new controller origin, not its absolute offset.
                control.reset_clutches()
                held = dict(zip(("left", "right"), fk(model, data, ik.L_hand_id, ik.R_hand_id, q)))
                q, _ = control.update(dict(left=frame((4, 5, 6)), right=frame((-4, -5, -6))),
                                      dict(left=True, right=True), q)
                for side in held:
                    np.testing.assert_allclose(control.targets[side][:3, 3], held[side][:3, 3], atol=1e-9)

    def test_pinocchio_permutation_and_nonfinite_solve(self):
        with patch("sys.argv", ["bridge"]):
            args = parse_args()
        spec = get_g1_embodiment("g1_23")
        ik = spec.make_ik()
        q = np.zeros(10)
        left, right = fk(ik.reduced_robot.model, ik.reduced_robot.data, ik.L_hand_id, ik.R_hand_id, q)
        # Deliberately non-identity mapping detects masks accidentally built in motor order.
        ik._arm_reorder_pin_to_g1 = np.arange(10)[::-1]
        control = ArmTargets(ik, spec.arm_index, dict(left=left, right=right), args)
        with patch.object(ik, "solve_ik", return_value=(np.ones(10), None)):
            result, _ = control.update(dict(left=frame()), dict(left=True), q)
        np.testing.assert_array_equal(result[:5], 0.)
        np.testing.assert_array_equal(result[5:], 1.)
        with patch.object(ik, "solve_ik", return_value=(np.full(10, np.nan), None)):
            result, _ = control.update(dict(left=frame()), dict(left=True), q)
        np.testing.assert_array_equal(result, q)

    def test_adapter_reads_once_and_isolates_incomplete_controller(self):
        with patch("sys.argv", ["bridge"]):
            args = parse_args()
        sys.path.append(args.isaacteleop_site_packages)
        from unitree_g1_lerobot.xr.both_controllers import BothXRControllers, Index
        device = BothXRControllers.__new__(BothXRControllers)
        device._external_inputs = {}
        device._running_events = Mock(return_value=[])
        valid = {Index.GRIP_POSITION: [1., 2., 3.], Index.GRIP_ORIENTATION: [0., 0., 0., 1.],
                 Index.SQUEEZE_VALUE: .9, Index.TRIGGER_VALUE: .2}
        device._step = Mock(return_value={"left": valid, "right": {Index.GRIP_POSITION: [4., 5., 6.]}})
        action = device.get_action()
        device._step.assert_called_once()
        self.assertEqual(device.tracking_by_hand, dict(left=True, right=False))
        self.assertTrue(device.is_tracking)
        self.assertEqual(action["left.squeeze"], .9)
        self.assertEqual(action["right.squeeze"], 0.)
        np.testing.assert_array_equal(action["right.grip_pos"], np.zeros(3))
        self.assertEqual(set(action), set(device.action_features))
        device._step.return_value = dict(left=None, right=valid)
        action = device.get_action()
        self.assertEqual(device.tracking_by_hand, dict(left=False, right=True))
        self.assertEqual(action["right.squeeze"], .9)


if __name__ == "__main__":
    unittest.main()
