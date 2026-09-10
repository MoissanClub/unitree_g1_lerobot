import unittest
import numpy as np

from unitree_g1_lerobot.diagnostics.verify_live_control import joint_result, pose_errors, trajectory_result


class LiveControlMetricsTests(unittest.TestCase):
    def test_trajectory_requires_motion_on_each_active_arm(self):
        rows = []
        for value in np.linspace(0, .1, 30):
            rows.append(dict(case="cartesian/both/0", command=[value, value], measured=[value, value],
                **{kind: [dict(position_m=0., rotation_rad=0.)]*2 for kind in ("ik", "actuator", "total")}))
        self.assertTrue(trajectory_result(rows)["passed"])
        for row in rows:
            row["measured"][1] = 0
        self.assertFalse(trajectory_result(rows)["passed"])

    def test_joint_direction_error_and_crosstalk(self):
        base = np.zeros(3)
        target = np.array([.12, 0., 0.])
        def result(q):
            return joint_result([dict(measured=q)]*10, base, target, 0)
        self.assertTrue(result(target)["passed"])
        self.assertFalse(result(-target)["passed"])
        self.assertFalse(result([0, .12, 0])["passed"])
        self.assertFalse(result([.12, .10, 0])["passed"])
        self.assertFalse(result([0, 0, 0])["passed"])

    def test_pose_translation_and_rotation_separate(self):
        target = np.eye(4)
        actual = np.eye(4)
        actual[0, 3] = .03
        actual[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        result = pose_errors([target], [actual])[0]
        self.assertAlmostEqual(result["position_m"], .03)
        self.assertAlmostEqual(result["rotation_rad"], np.pi/2)
