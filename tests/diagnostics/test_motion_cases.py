"""Check that extracting baseline cases preserves the existing trajectories."""
import unittest

import numpy as np
import pinocchio as pin

from unitree_g1_lerobot.diagnostics.shared.motion_cases import baseline_pose, cartesian_targets


class MotionCaseTests(unittest.TestCase):
    def test_baseline_for_both_embodiments(self):
        from unitree_g1_lerobot.robots.unitree_g1 import get_g1_embodiment

        for embodiment in ("g1_29", "g1_23"):
            names = [joint.name for joint in get_g1_embodiment(embodiment).arm_index]
            q = baseline_pose(names)
            expected = [-.4, .15, 0., .8, 0.] + ([0., 0.] if embodiment == "g1_29" else [])
            right = expected.copy()
            right[1] = -.15
            np.testing.assert_array_equal(q, expected + right)

    def test_all_original_sweep_samples_and_inactive_hand(self):
        homes = [np.eye(4), np.eye(4)]
        homes[0][:3, :3] = pin.exp3(np.array([.1, -.2, .3]))
        homes[1][:3, 3] = [.3, -.2, .1]
        original = [p.copy() for p in homes]
        for side in ("left", "right", "both"):
            for axis in range(6):
                for step in range(90):
                    targets = cartesian_targets(homes, side, axis, step)
                    phase = np.sin(2*np.pi*step/89)
                    for hand in range(2):
                        expected = homes[hand].copy()
                        if side == "both" or hand == (0 if side == "left" else 1):
                            if axis < 3:
                                expected[axis, 3] += .015*phase
                            else:
                                rotation = np.zeros(3)
                                rotation[axis-3] = .08*phase
                                expected[:3, :3] = homes[hand][:3, :3] @ pin.exp3(rotation)
                        np.testing.assert_array_equal(targets[hand], expected)
        np.testing.assert_array_equal(homes, original)

    def test_invalid_case_rejected(self):
        for side, axis, step in (("invalid", 0, 0), ("both", 6, 0), ("left", 0, 90)):
            with self.assertRaises(ValueError):
                cartesian_targets([np.eye(4), np.eye(4)], side, axis, step)
