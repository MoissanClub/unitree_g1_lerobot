"""Shared G1 arm kinematics and command helpers; no XR dependency."""
from __future__ import annotations

import time

import numpy as np
import pinocchio as pin


def fk(model: pin.Model, data: pin.Data, left_frame_id: int, right_frame_id: int, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return data.oMf[left_frame_id].homogeneous.copy(), data.oMf[right_frame_id].homogeneous.copy()


def action_from_arm_q(q_g1: np.ndarray, joint_index: type, arm_index: type) -> dict[str, float]:
    action = {f"{joint.name}.q": 0.0 for joint in joint_index}
    action.update({f"{joint.name}.q": float(q_g1[i]) for i, joint in enumerate(arm_index)})
    return action


def scale_arm_gains(robot, arm_index: type, kp_scale: float, kd_scale: float) -> None:
    if kp_scale <= 0.0 or kd_scale <= 0.0:
        raise ValueError("arm gain scales must be positive")
    if kp_scale == 1.0 and kd_scale == 1.0:
        return
    robot.kp = np.asarray(robot.kp, dtype=np.float32).copy()
    robot.kd = np.asarray(robot.kd, dtype=np.float32).copy()
    for joint in arm_index:
        robot.kp[joint.value] *= kp_scale
        robot.kd[joint.value] *= kd_scale
    print(f"scaled arm gains: kp x{kp_scale:.2f}, kd x{kd_scale:.2f}")


def make_ready_targets(
    left_home: np.ndarray,
    right_home: np.ndarray,
    ready_x_m: float,
    ready_z_m: float,
    ready_spread_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    left_ready = left_home.copy()
    right_ready = right_home.copy()
    left_ready[:3, 3] += np.array([ready_x_m, ready_spread_m, ready_z_m])
    right_ready[:3, 3] += np.array([ready_x_m, -ready_spread_m, ready_z_m])
    return left_ready, right_ready


def solve_ready_q(ik, left_ready: np.ndarray, right_ready: np.ndarray, q_seed: np.ndarray) -> np.ndarray:
    q = np.asarray(q_seed, dtype=float)
    for _ in range(12):
        q, _ = ik.solve_ik(left_ready, right_ready, q)
        q = np.asarray(q, dtype=float)
    if not np.all(np.isfinite(q)):
        raise RuntimeError("Ready pose IK produced non-finite values")
    return q


def publish_ready_for(robot, action: dict[str, float], duration_s: float, hz: float) -> None:
    if robot is None:
        time.sleep(max(0.0, duration_s))
        return
    period_s = 1.0 / hz
    deadline = time.monotonic() + max(0.0, duration_s)
    while time.monotonic() < deadline:
        t0 = time.perf_counter()
        robot.send_action(action)
        time.sleep(max(0.0, period_s - (time.perf_counter() - t0)))
